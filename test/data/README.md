# Data for development

- Every set of data consists of
  - A pcap file - primary capture
  - A NMEA/SBF file - logged from RxTools
  - A screenshot - Indicating the data captured (fields selected in the webinterface)

* For clarity, all nmea captures are even numbered and sbf odd numbered

## Different Message options as seen in the web interface

### SBF

* **Measurements**
    * MeasEpoch
    * MeasExtra
    * EndOfMeas

* **Meas3**
    * Meas3Ranges
    * Meas3CN0HiRes
    * Meas3Doppler
    * Meas3PP
    * Meas3MP

* **RawNavBits**
    * GPSRawCA
    * GPSRawL2C
    * GPSRawL5
    * GLORawCA
    * GALRawFNAV
    * GALRawINAV
    * GEORawL1
    * GEORawL5
    * BDSRaw
    * QZSRawL1CA
    * QZSRawL2C
    * QZSRawL5
    * NAVICRaw
    * BDSRawB1C
    * BDSRawB2a

* **GPS**
    * GPSNav
    * GPSAlm
    * GPSIon
    * GPSUtc

* **GLO**
    * GLONav
    * GLOAlm
    * GLOTime

* **GAL**
    * GALNav
    * GALAlm
    * GALIon
    * GALUtc
    * GALGstGps
    * GALSARRLM

* **GEO**
    * GEOMT00
    * GEOPRNMask
    * GEOFastCorr
    * GEOIntegrity
    * GEOFastCorrDegr
    * GEONav
    * GEODegrFactors
    * GEONetworkTime
    * GEOAlm
    * GEOIGPMask
    * GEOLongTermCorr
    * GEOIonoDelay
    * GEOServiceLevel
    * GEOClockEphCovMatrix

* **BDS**
    * BDSAlm
    * BDSNav
    * BDSIon
    * BDSUtc

* **QZS**
    * QZSNav
    * QZSAlm

* **PVTCart**
    * PVTCartesian
    * PosCovCartesian
    * VelCovCartesian
    * BaseVectorCart

* **PVTGeod**
    * BaseVectorGeod
    * PVTGeodetic
    * PosCovGeodetic
    * VelCovGeodetic
    * PosLocal

* **PVTExtra**
    * DOP
    * EndOfPVT
    * PVTSupport
    * PVTSupportA

* **Attitude**
    * AttEuler
    * AttCovEuler
    * EndOfAtt

* **Time**
    * ReceiverTime
    * xPPSOffset

* **Event**
    * ExtEventAttEuler
    * ExtEvent
    * ExtEventPVTCartesian
    * ExtEventPVTGeodetic
    * ExtEventBaseVectGeod

* **DiffCorr**
    * DiffCorrIn
    * BaseStation
    * RTCMDatum

* **Status**
    * OutputLink
    * InputLink
    * SatVisibility
    * ChannelStatus
    * ReceiverStatus
    * IPStatus
    * QualityInd
    * NTRIPClientStatus
    * DiskStatus
    * RFStatus
    * DynDNSStatus
    * P2PPStatus

* **LBand**
    * LBandTrackerStatus
    * LBAS1DecoderStatus
    * LBAS1Messages
    * LBandBeams

* **UserGroups**
    * Group1
    * Group2
    * Group3
    * Group4

* PosCart

* ReceiverSetup

* Commands

* Comment

* BBSamples

* ASCIIIn

* PosProjected

* RxMessage

### NMEA

