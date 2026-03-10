# Summary

1. [Remote Desktop Access to Raspberry Pi](#remote-desktop-access-to-raspberry-pi)
2. [Remote Terminal access via SSH](#remote-terminal-access-via-ssh)
3. [Remote File exploration from your File Explorer via SFTP](#remote-file-exploration-from-your-file-explorer-via-sftp)

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

## Remote Terminal access via SSH

To connect directly to the rasberry in SSH :

1. You must be on the same (wifi) network as the rasberry pi

2. In terminal do :

```bash
ssh voiture@10.42.0.1
```

password : **ros**

On first connection or in case of an issue with the ssh key :

```bash
ssh-keygen -R 10.42.0.1
```

Then try connecting again.

## Remote File exploration from your File Explorer via SFTP

You can explore the Raspberry Pi's folders in your File Explorer (and use VSCode on your computer) by openning an SFTP connection from your File Explorer.

1. SFTP uses an SSH connection : check that you can open an ssh connection.

2. Look on the web how to connect to an SFTP server (also called SSH-FTP) using your File Explorer. It depends on the File Explorer you use.

3. When configuring the client, use :

   ```txt
   Protocol : SFTP
   Server : 10.42.0.1
   User : voiture
   Port : 22 (default)
   password : ros
   ```

If multiple people are editing files on the Raspberry Pi at the same time, make sure to **refresh** the file explorer manually.

It will look like the Raspberry Pi is mounted on your machine like a USB drive, if you want to actually do that, use **SSHFS** (see on the web).
