//
//        3D Image Sensor by PEW 
//
//-- 2010.07.17 -- 
// DLL function decralation
extern "C" {
int		__stdcall InitImageDriver	( void );
int		__stdcall FreeImageDriver	( void );
int		__stdcall GetImageKN		( unsigned short *kdat, unsigned short *ndat );
char*	__stdcall GetDLLversion		( void );

int __stdcall ChangeFreq	( int freq_pat );
int __stdcall Freqmode		( void );
int __stdcall ChangeSleep	( int sleep_pat );
int __stdcall Sleepmode		( void );
int __stdcall InitPara		( void );
int __stdcall ChangeSpeed   ( int speed_pat );
int __stdcall Speedmode     ( void );

}
