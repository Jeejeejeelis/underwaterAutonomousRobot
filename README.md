# underwaterAutonomousRobot
## Signal Flow Diagram
<img width="842" alt="Signal flow diagram" src="https://github.com/user-attachments/assets/1a63130f-e5be-4e8b-9620-3fa6df868ee5" />


### Notes
- **test_gnss**: Not intended for the sim7600X.
- **test_ctd**: Contains TODO parts related to magic numbers and depth calculations. These relate to a file `ctd.c` which is currently missing.
- **SIM7600X-4G-HAT-Demo**: this is currently used to test GNSS on the sim7600x. Files on rpi, to be uploaded here.
- Argofloat profiling data? 

### TODO
- Address magic numbers in `test_ctd`.
- Implement depth calculations in `test_ctd`.
- Github repo with underwater simulation. https://github.com/tervoju/argofloat 