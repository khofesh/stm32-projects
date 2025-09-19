- write collect-data application, this application will collect data from sensors and save it to timescaledb
  systemd-timer will run the systemd-service that will run this collect-data application
  systemd-will trigger every 15 minutes and run collect-data application for 2 minutes
