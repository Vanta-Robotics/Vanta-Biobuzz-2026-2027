FTC DASHBOARD — SIMPLE CHECKLIST

SETUP
1. Deploy the updated TeamCode app to the Control Hub using Android Studio's Run button.
2. Connect your laptop to the robot's Wi-Fi. Ignore "No Internet."
3. Open http://192.168.43.1:8080/dash
4. On the Driver Station, select Main TeleOp, then INIT and Start.

GRAPH
5. In Dashboard's Graph panel, select:
   Shooter target (RPM)
   Shooter actual (RPM)
   Click Start Graphing.

TUNE
6. Under Configuration > Shooter, set KP=0, KI=0, KD=0, KF=1. Press Enter after each edit.
7. Test without material first. Leave the bumpers alone. Hold the right trigger steadily and let speed settle.
8. Actual RPM too low? Increase KF slightly. Too high? Decrease KF slightly.
   Release the trigger before each change. Press Enter to apply it, then test again.
9. Keep your best KF. Increase KP slowly from 0 to improve response. If RPM swings repeatedly, reduce KP.
   Leave KI and KD at 0 for now.

SAVE
10. Write down the good gains. Release the trigger and press STOP.
11. Put those four values into Shooter.java, then rebuild and deploy.
    Dashboard edits do not save into your code automatically.

IF SOMETHING IS WRONG
- Page won't load: check robot Wi-Fi and that the updated app was deployed.
- No graph readings: Main TeleOp must be started, not just initialized.
- Motor spins but RPM stays zero or is negative: STOP and check the encoder/configuration.
- Power stays at 1 but speed is too low: do not keep increasing gains; check the battery, load, and target.

Full trigger currently requests 50% of the configured motor's achievable speed; output power can be higher.
