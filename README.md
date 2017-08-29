OSGAR
=====

Open Source Garden (Autonomous) Robot

![John Deere X300R](http://robotika.cz/competitions/roboorienteering/2016/jd-nav2.jpg)

References at
http://robotika.cz/robots/osgar/

Video: https://youtu.be/KiDnPsnLmLU

# Notes/Howto

To run demo with four cones use:

```
python ./navpat.py run config/playground15x5.json
```

To replay existing log file use meta log, for example:
```
python ./navpat.py replay logs/meta_160821_160615.log
```

Current visualization is using **Eduro Viewer**.
Add extra parameter **--view** in log replay to see the data
```
python ./navpat.py replay logs/meta_160821_160615.log --view
```

For more options see
```
python ./navpat.py --help
```

