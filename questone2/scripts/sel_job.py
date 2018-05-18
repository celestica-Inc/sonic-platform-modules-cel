import syslog
import subprocess
import os
import sys
import re
from datetime import datetime

def run_command(command):
    try:
        proc = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
        (out, err) = proc.communicate()           
    except IOError as e:
        sys.exit(0)     
    return out

def add_syslog(message):
    # Add log to syslog
    syslog.syslog(syslog.LOG_WARNING, message)

def get_sel():
    # Get event from BMC
    sel_str = run_command('ipmitool sel list')
    return sel_str.splitlines()

def get_bmc_time():
    # Get BMC time
    bmc_time = run_command('ipmitool sel time get')
    return str(bmc_time)

def diff_time_min(stop, start):
    # Get difference of time in minutes
    td = stop - start
    return int(round(td.total_seconds() / 60))

def sync_time():
    # Sync time between BMC and SONiC
    bmc_time = get_bmc_time().rstrip('\n')
    bmc_datetime = datetime.strptime(bmc_time, '%m/%d/%Y %H:%M:%S')
    td_mins = diff_time_min(datetime.now(), bmc_datetime)
    if td_mins != 0:
        run_command('ipmitool sel time set now')

def get_exist_log():
    sel_log = run_command('cat /var/log/syslog | grep sel_job.py')
    
    sel_log_list = sel_log.splitlines() if sel_log else []
    exist_timestamp = []
    for str_data in sel_log_list:
        date_string = re.search(r'\d{2}/\d{2}/\d{4} \d{2}:\d{2}:\d{2}', str_data)
        sel_datetime = datetime.strptime(date_string.group(), '%m/%d/%Y %H:%M:%S')
        exist_timestamp.append(sel_datetime)
    return exist_timestamp

def main():
    try:
        # Check datetime
        sync_time()
	# Check existing log
        ex_log_ts = get_exist_log()
        # Check event
        sel_list = get_sel()
        sel_temp_list = matching = [s for s in sel_list if "Temperature" in s]
        for sel_temp in sel_temp_list:
            sel_temp_info_list = [x.strip() for x in sel_temp.split('|')] 
            date, time, title, message = sel_temp_info_list[1:5]
            sel_datetime = datetime.strptime(date+time, '%m/%d/%Y%H:%M:%S')
            if sel_datetime not in ex_log_ts:
                add_syslog("BMC WARNING : "+ date + " " + time +" "+ title + " " +  message)
    except Exception as e:    
        sys.exit(0)

if __name__ == '__main__':
    main()
