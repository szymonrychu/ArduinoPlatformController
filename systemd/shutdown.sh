echo "waiting" > /var/run/shutdown_signal
while inotifywait -e close_write /var/run/shutdown_signal; do 
  signal=$(cat /var/run/shutdown_signal)
  if [ "$signal" == "true" ]; then 
    echo "done" > /var/run/shutdown_signal
    shutdown -h now
  fi
done