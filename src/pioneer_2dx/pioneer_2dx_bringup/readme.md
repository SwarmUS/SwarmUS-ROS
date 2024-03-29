## Systemd service
You can use the provided swarmus-pioneerX.service to start the ros nodes. Make sure your bashrc sources the catkin_ws, export the required variables. Also make sure it can be executed even when non-interactive.

To add the service you can use those commands.
```sh
sudo cp swarmus-pioneer1.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable swarmus-pioneer1
```
Copy the service `swarmus-pioneer2.service` for the pioneer 2.

Then you can reboot or start the service manually using `sudo start enable swarmus-turtlebot`.

To be runable when non-interactive, you can remove those lines in your .bashrc if they are present.
```sh
case $- in
    *i*) ;;
      *) return;;
esac
```

You should only copy the right pioneer service. Use pioneer1 if the camera mount is purple and pioneer2 if the camera mount is orange.
