#!/bin/sh
PRO_NAME=/mnt/SDCARD/menred

  
while true ; do
  
  NUM=`ps | grep ${PRO_NAME} | grep -v grep |wc -l`

#  echo $NUM
#    少于1，重启进程
  if [ "${NUM}" -lt "1" ];then
    #echo "${PRO_NAME} was killed"
        rm /root/gflag
        #/mnt/SDCARD/menred -qws &
	reboot
#    大于1，杀掉所有进程，重启
  elif [ "${NUM}" -gt "1" ];then
    #echo "more than 1 ${PRO_NAME},killall ${PRO_NAME}"
    killall -9 $PRO_NAME
    rm /root/flag
     #/mnt/SDCARD/menred -qws &
     reboot 
 fi
#    kill僵尸进程
  NUM_STAT=`ps aux | grep ${PRO_NAME} | grep T | grep -v grep | wc -l`
  
  if [ "${NUM_STAT}" -gt "0" ];then
    killall -9 ${PRO_NAME}
    rm /root/gflag
     #/mnt/SDCARD/menred -qws &
     reboot 
 fi
sleep 1
done
  
exit 0
