
# Notes on xArm Communications Protocol

The xArm connects to a laptop over USB, which is presented as a kind of USB-HID
(human interface device, which is commonly used for things like keyboards and
mice). USB-HID is a packet-oriented protocol: the laptop sends a blob of bytes
as a message, robot sends back a blob of bytes as a response. The messages from
laptop to robot all follow a common format, as do the responses from robot to
laptop. The messages are all raw binary (bytes), not ascii text. 

Note: In USB-HID terminology, a message is called a "report". USB-HID reports
have a report-ID, but these are not something we care about. Sometimes the
report-ID will get put on the start of a message, but we don't care what value
it has, and should ignore it. We only care about the contents of the reports.

# Generic message between laptop and robot, in either direction

Every message to the robot contains a _payload_, which starts with a 1-byte
_command_ optionally followed by some more bytes for _parameters_. The _command_
might be 3 to mean "move servos", and the _parameters_ would be info about the
position(s), speed, etc. Or _command_ might be 21 for "query position", then
_parameters_ is info about which motor(s) to query.

Responses from the robot are the same, but they don't have the _command_, the
_payload_ just has whatever response values there are.

The full message has a 2-byte prefix consisting of 0x55, 0x55, then a length
byte, followed by the _payload_. The length byte doesn't count the prefix, so it
is 1 more than the length of the payload.

    [   0x55,   0x55,  _LEN_,  _CMD_, ... more bytes here for the parameters ]
    | <-- prefix --> |        | <--------- this part is the payload -------> |
    | <-- prefix --> |  <--------- this part is _LEN_ bytes long ----------> |

Some parameters are 1-byte unsigned values, from 0 to 255.
Some parameters are 1-byte signed values, from -128 to +127.
Some parameters are 2-byte unsigned values, from 0 to 65535. These are encoded
with the low order byte first, then the high order byte.
Examples:
* The two decimal bytes [ 1, 3 ] encode the value 3x256 + 1 = 769 decimal, or 0x0301 in hex.
* The two hex bytes [ 0x75, 0x34 ] encode the value 0x3475 in hex, or 13429 in decimal

# Request to move motor(s)

Parameters:
 _N_ is the number of motors to move, a 1-byte value from 1-255
     (no idea what happens if you use _N_ = 0 here)
 _T_ is the time, a duration in milliseconds, a 2-byte value from 1-32767
     (no idea what happens if _T_ is larger, perhaps it is treated as negative?)
 _i1_, ..., _iN_ is a list of servo motor IDs, each is a 1-byte values from 1-255
     (no idea what happens if you use an invalid motor ID)
 _p1_, ..., _pN_ is a list of positions, each is a 2-byte value from 1-32767
     (no idea what happens if _T_ is larger, perhaps it is treated as negative?)

There should be _N_ IDs, and _N_ positions.

The message is encoded as:

    [ 0x55, 0x55, _LEN_, _CMD_, _N_, _T_(lo), _T_(hi), _i1_, _P1_(lo), _P1_(hi), ... _iN_, _PN_(lo), _PN_(hi) ]

 _LEN_ is the total number of bytes in the payload, NOT INCLUDING the 0x55, 0x55, _LEN_ part.
 _CMD_ is 3 for "move servos".

Example:

    [ 0x55, 0x55,  7,    3,        1,       0, 5,         2,           0, 3       ]
      HEADER WITH LEN  "move"  one motor   5*256+0 ms   servo#2    3*256+0 units
    

# Query to get motor position(s)

Parameters:
 _N_ is the number of motors to be queried, a 1-byte value from 1-255
     (no idea what happens if you use _N_ = 0 here)
 _i1_, ..., _iN_ is a list of servo motor IDs, each is a 1-byte values from 1-255
     (no idea what happens if you use an invalid motor ID)

There should be _N_ IDs.

The message is encoded as:

    [ 0x55, 0x55, _LEN_, _CMD_, _N_, _i1_, ... _iN_ ]

 _LEN_ is the total number of bytes in the payload, NOT INCLUDING the 0x55, 0x55, _LEN_ part.
 _CMD_ is 21 for "query servo position".

Example:

    [ 0x55, 0x55,  3,    21,        1,         2     ]
      HEADER WITH LEN  "query"  one motor   servo#2

# Response from query to get motor position(s)

The response is encoded as:

    [ 0x55, 0x55, _LEN_, _N_, _i1_, _P1_(lo), _P1_(hi), ... _iN_, _PN_(lo), _PN_(hi) ]

Where:
 _N_ is the number of motors included in the response
 _i1_, ..., _iN_ is a list of servo motor IDs, each is a 1-byte values from 1-255
 _p1_, ..., _pN_ is a list of positions, each is a 2-byte value from 1-32767

Example:

    [ 0x55, 0x55,  7,    1,          2,          0, 3     ]
      HEADER WITH LEN  one motor  servo#2    3*256+0 units

