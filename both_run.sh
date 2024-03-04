ipcs | grep -q 0x00654321
if [ $? -eq 0 ]; then
  ipcrm -M 0x00654321
fi
ipcs | grep -q 0x00654322
if [ $? -eq 0 ]; then
  ipcrm -M 0x00654322
fi
ipcs | grep -q 0x00123456
if [ $? -eq 0 ]; then
  ipcrm -S 0x00123456
fi
ipcs | grep -q 0x00123457
if [ $? -eq 0 ]; then
  ipcrm -S 0x00123457
fi

sh ./con_run.sh | sh ./ai_run.sh