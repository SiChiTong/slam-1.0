#include "CTimestamp.h"

using namespace std;

CTimestamp::CTimestamp()
{

}

CTimestamp::CTimestamp(bool bReadBaseTimestamp)
{
	
}

double CTimestamp::getTimestamp(double  fSecTimeScale , double fUSecTimeScale)
{
	if (gettimeofday( &msTime , NULL) == -1 )
	{
		cout << " could not get time of day!";
		return 0;
	}
	msTime.tv_sec -= mfBaseTimestamp / 1000000;
//	cout << msTime.tv_sec  << " " << msTime.tv_usec << endl;
	return (double)msTime.tv_sec * fSecTimeScale + (double)msTime.tv_usec / fUSecTimeScale ;
}

double CTimestamp::getTimestampMicro()
{
	return getTimestamp( 1000000 , 1);
}

double CTimestamp::getTimestampMilli()
{
	return getTimestamp( 1000 , 1000);
}

double CTimestamp::getTimestampSec()
{	
	return getTimestamp( 1 , 1000000);
}



double CTimestamp::getTimeDiff(double  fTimestamp , double fOldTimestamp)
{
	return fTimestamp - fOldTimestamp;
}

double CTimestamp::getBaseTimestamp()
{
	if (gettimeofday( &msTime , NULL) == -1 )
        {
                cout << " could not get time of day!";
                return 1200000;
        }
        return (double)msTime.tv_sec * 1000000 + (double)msTime.tv_usec; 
}
