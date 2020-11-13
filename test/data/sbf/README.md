# Driver progress test

## Test better\_publishing branch
============================================================================

### NMEA
----------------------------------------------------------------------------

* Selected streams: GGA, GSA, GSV, RMC, VTG
* Topics found: /nav\_sat\_fix and /velocity

Other remarks: constant output of "not enough msg" on console (driver node)

#### rostopic echo 

* navsatfix - ok
* velocity - no output

Exact network packets saved as nov2-nmea-set1.pcap
Screenshot attached as nov2-nmea-set1.png

### SBF
----------------------------------------------------------------------------

* Selected streams: all
* Topics found: /nav\_sat\_fix and /velocity

Other remarks: constant output of "not enough msg" on console (driver node)

#### rostopic echo 

* navsatfix - ok
* velocity - no output

Exact network packets saved as nov2-sbf-set2.pcap
Screenshot attached as nov2-sbf-set2.png

## Test response to commands
============================================================================

Stream type used: SBF
Selected streams: PostProcess and Support
Command send: "grc"

### Network packets

* nov2-sbf-set3.pcap

	* Filtered by : Source IP only

	* This will noisy data from source IP (192.168.3.1) to my PC (192.168.3.41) as this is not filtered by port (Module was streaming data on port 9999)
	


* nov2-sbf-set4.pcap
	
	* Filtered by : Source IP and Port

	* This contains data only from module IP on port 9999

#### Remarks

* Screenshot attached as nov2-sbf-set3.png

* No instance of $! characters was found in the ascii form of both pcap files :(


