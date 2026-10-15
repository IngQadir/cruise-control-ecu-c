| ID     | Requirement                                                              |
|--------|--------------------------------------------------------------------------|
| REQ-01 | The system shall be in OFF state after start-up.                         |
| REQ-02 | The system shall enter ACTIVE when SET is pressed and speed is between   |
|        | 30 and 180 km/h.                                                         |
| REQ-03 | The system shall switch to OFF when the brake pedal is pressed.          |
| REQ-04 | The system shall limit the throttle request to 0..100 %.                 |
| REQ-05 | The system shall enter FAULT if the speed signal is invalid.             |
| REQ-06 | The system shall switch to OFF when CANCEL is pressed.                   |
| REQ-07 | The system shall leave ACTIVE if speed drops below 30 km/h.              |
| REQ-08 | The system shall move from OFF to STANDBY when the main switch is on. |
| REQ-09 | The system shall leave FAULT only when the speed signal is valid and the main switch is off. |
| REQ-10 | The system shall encode vehicle speed with 16 bits and a resolution of 0.01 km/h. |
| REQ-11 | The system shall encode the throttle request with 8 bits and a resolution of 0.5 %. |
| REQ-12 | The system shall limit signal values to the representable raw range. |
| REQ-13 | The system shall reject signal layouts that do not fit in the frame. |
