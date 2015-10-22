#ifndef CTIMESTAMP_H
#define CTIMESTAMP_H

#include <sys/time.h>
#include <iostream>


class CTimestamp
{
private:
struct timeval msTime ;
double mfBaseTimestamp; //this value gets set from the Repository
double getTimestamp(double fSecTimeScale , double fUSecTimeScale);

public:
CTimestamp();
CTimestamp(bool bReadBaseTimestamp);


double getBaseTimestamp();
double getTimestampMicro();
double getTimestampMilli();
double getTimestampSec();
double getTimeDiff(double fTimestamp , double fOldTimestamp );
};

#endif
