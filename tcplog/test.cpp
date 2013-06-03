
#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <errno.h>
#include <winsock.h>
#include <conio.h>
#include <string>
#include <time.h>
#include <iostream>

using namespace std;



string GetNowFilename()
{

	time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://www.cplusplus.com/reference/clibrary/ctime/strftime/
    // for more information about date/time format
	sprintf(buf,"LOG%02d-%02d-%04d-%02d:%02d.net",tstruct.tm_mon,tstruct.tm_mday,tstruct.tm_year+1900,tstruct.tm_hour,tstruct.tm_min);
	string s=buf;
	return s;
}


int main()
{
string s=GetNowFilename();
cout<<"Name:["<<s<<"]"<<endl;

return 0;
}
