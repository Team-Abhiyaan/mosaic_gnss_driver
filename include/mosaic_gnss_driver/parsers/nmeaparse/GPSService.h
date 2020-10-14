
#ifndef GPSSERVICE_H_
#define GPSSERVICE_H_

#include <string>
#include <chrono>
#include <functional>
#include <mosaic_gnss_driver/parsers/nmeaparse/GPSFix.h>
#include <mosaic_gnss_driver/parsers/nmeaparse/NMEAParser.h>
#include <mosaic_gnss_driver/parsers/nmeaparse/Event.h>

namespace nmea {

class GPSService {
private:

	void read_PSRF150(const NMEASentence& nmea);
	void read_GPGGA	(const NMEASentence& nmea);
	void read_GPGSA	(const NMEASentence& nmea);
	void read_GPGSV	(const NMEASentence& nmea);
	void read_GPRMC	(const NMEASentence& nmea);
	void read_GPVTG	(const NMEASentence& nmea);

public:
	GPSFix fix;

	GPSService(NMEAParser& parser);
	virtual ~GPSService();

	Event<void(bool)> onLockStateChanged;		// user assignable handler, called whenever lock changes
	Event<void()> onUpdate;						// user assignable handler, called whenever fix changes

	void attachToParser(NMEAParser& parser);			// will attach to this parser's nmea sentence events
};


}

#endif /* GPSSERVICE_H_ */
