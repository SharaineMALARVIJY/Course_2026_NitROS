# Summary

[Remote Desktop Access to Raspberry Pi](#remote-desktop-access-to-raspberry-pi)

## Remote Desktop Access to Raspberry Pi

1. Download and install TigerVNC Viewer on your PC.
2. Connect your PC to the Raspberry Pi's Wi-Fi network (password: `setup1234`). 
3. SSH from your PC (password: `ros`):
```bash
ssh -L 5901:localhost:5901 voiture@10.42.0.1
```

4. Start VNC on Raspberry Pi (inside SSH):

```bash
vncserver
```

5. Download and use TigerVNC Viewer on your PC:
   Connect to `localhost:5901` with password `setup1234`.
6. If needed, the password of the Raspberry Pi is: `ros`.
