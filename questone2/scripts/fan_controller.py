import subprocess
import click
import os
import sys
import time

def run_command(command, echo = True):
    proc = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    (out, err) = proc.communicate()

    try:
	if echo:
            click.echo(out)             
    except IOError as e:
        if e.errno == errno.EPIPE:
            sys.exit(0)
        else:
            raise

    if proc.returncode != 0:
        sys.exit(proc.returncode)
    
    return out

@click.group()
def cli():
    """fancontrol_util - Command line utility for monitor and control fan speed"""

    if os.geteuid() != 0:
        click.echo("Root privileges are required for this operation")
        sys.exit(1)

# 'version' subcommand
@cli.command()
def version():
    """Display version info"""
    click.echo("Version 1.0")

# 'status' subcommand
@cli.command()
def status():
    """Show fan status"""
    fcs_status = run_command('ipmitool raw 0x3a 3 0', echo = False)
    if int(fcs_status):
        click.echo("Fan control system : AUTO")
    else:
        click.echo("Fan control system : MANUAL")
    run_command('ipmitool sdr | grep Fan | grep RPM')

#
# 'platform' group ("fancontrol set ...")
#

@cli.group()
def set():
    """Fan setting"""
    pass

# 'mode' subcommand ("fancontrol set mode")
@set.command()
@click.argument('mode', type=click.Choice(['auto', 'manual']), required=True)
def mode(mode):
    """set Fan control mode"""
    if mode == 'auto':
    	run_command('ipmitool raw 0x3a 3 1 1', echo = False)
    else:
    	run_command('ipmitool raw 0x3a 3 1 0', echo = False)
    
    fcs_status = run_command('ipmitool raw 0x3a 3 0', echo = False)
  
    if mode == 'auto' and int(fcs_status) == 1:
    	click.echo("Fan control system : AUTO")
    elif mode == 'manual' and int(fcs_status) == 0:
	click.echo("Fan control system : MANUAL")
    else:
	click.echo("Error") 

# 'pwm' subcommand ("fancontrol set pwm")
@set.command()
@click.argument('pwm', type=click.IntRange(0, 100), required=True)
@click.option('-p', '--psu', is_flag=True)
@click.option('-c', '--chassis', is_flag=True)
def pwm(pwm, psu, chassis):
    """set fan speed"""
    fcs_status = run_command('ipmitool raw 0x3a 3 0', echo = False)
    if int(fcs_status) == 1:
	click.echo("Fan control system : AUTO")    
	sys.exit(0)

    if chassis:
	run_command('ipmitool raw 0x3a 3 2 1 '+ str(pwm), echo = False)
    elif psu:
	run_command('ipmitool raw 0x3a 3 2 2 '+ str(pwm), echo = False)

    if not chassis and not psu:
	run_command('ipmitool raw 0x3a 3 2 3 '+ str(pwm), echo = False)

    time.sleep(3)
    run_command('ipmitool sdr | grep Fan | grep RPM')

if __name__ == '__main__':
    cli()
