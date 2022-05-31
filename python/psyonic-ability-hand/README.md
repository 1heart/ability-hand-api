# Psyonic Hand Interface

This is a [python-poetry](https://python-poetry.org/) based project. To install locally,
after installing poetry via the link, run
```
poetry install
```

## Configuration

Then hand comes default configured for I2C communications. However then hand needs to be
configured for UART mode communication (which may require a Psyonic firmware upgrade).

Also, most of the current code defaults to 230400 UART speed, whereas the hand defaults
to 460800.

## Running the demos

After installing with `poetry install`, run

```
poetry shell
```
to create a local python environment.

### hand-demo.py

```
python src/psyonic_ability_hand/hand-demo.py
```

This provides a basic interface for controlling each finger in position or velocity mode,
as well as viewing the touch data.

### hand-grasp-test.py

```
python src/psyonic_ability_hand/hand-grasp-test.py
```

A simpler tool for stepping through opening and closing the gripper


## Hand Interface

`hand.py` contains `Hand` object, a typical usage might be:

```py
from psyonic_ability_hand.hand import Hand
from psyonic_ability_hand.io import SerialIO

hand = Hand(SerialIO())

# packet transmission to and from the hand is asyncronous and must be started
# prior to sending commands
hand.start()

hand.set_position( JointData(0, 0, 0, 0, 0, -10) )

hand.stop()
```

Most of the data passed to the interface is in the form of the JointData object:

```py
class JointData:
    Index: float = 0
    Middle: float = 0
    Ring: float = 0
    Pinky: float = 0
    ThumbFlexor: float = 0
    #NOTE: the ThumbRotator is inverted, -10 is the lower limit, around -90 the upper
    ThumbRotator: float = 0
```

The basic commands are:

```py
get_position()
set_position( JointData() )
```
With positions, JointData represents angles in degress of each finger, from `0` to `150`
(with a practical limit for most fingers around `90`).

*NOTE: in tests, the hand, due to hardware/firmware limitations, may stop approaching a
position within a few degrees of the target, even if it's physically possible for it
to reach the exact target.*

The general 'smoothness' of position movements is also highly depending on the packet
transmission interval, currently 20ms. The hand will move in small incremenents towards
the target position with each packet sent.

```py
set_velocity( JointData() )
```

In velocity set mode, the `JointData` provides degrees per second for each finger.


Additionally, the status of the Touch Sensors can be retrieved via:

```
get_touch()
```
