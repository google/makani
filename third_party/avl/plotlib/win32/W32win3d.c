/*********************************************************************** 
   W32win.c - FORTRAN/C interface for Windows NT/95 Xplot11
 
    Copyright (C) 1999 Harold Youngren, Mark Drela 
 
    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Library General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Library General Public License for more details.

    You should have received a copy of the GNU Library General Public
    License along with this library; if not, write to the Free
    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

    Report problems to:    guppy@maine.rr.com 
                        or drela@henry.mit.edu  
***********************************************************************/ 

/***********************************************************************
*  Xplot11 C-level Win32 interface
C     Version 4.43  3/5/99
*
*  Status: This code is still pretty rough, it works but there are some
*          rough edges (some of which result from problems with the Win32
*          API).  This is the interface code to create, draw to, refresh,
*          destroy a graphics window on a Win32 machine running a Fortran
*          application (like XFOIL) making Pltlib calls.  
*
*          The refresh problem for Win32 (as the Xwindows option for a 
*          automatic refresh using the server's backing store startup
*          option is not available under Win32) is dealt with by creating
*          a plot thread that monitors the Windoze events and does window
*          refresh.  This is done by doing plotting to a memory bitmap and 
*          writing the bitmap to the screen whenever the window needs 
*          refresh.
*
*          Note that several problems arise in supporting the Xwindows
*          plotting functionality under Win32.  Colors are handled differently
*          than under Xwindows, for color name compatibility I have included
*          a color table lookup routine that maps Xwindows color names into
*          RGB values.  A more serious problem is the inconsistency of the 
*          Win32 API itself, specifically the functionality varies with the
*          version of the Microsoft OS you are running.  The most serious
*          problem showed up in drawing lines with arbitrary patterns and 
*          widths.  The Win32 API apparently does not support drawing these
*          types of lines (used in Pltlib for background grids, among other
*          things) satisfactorily.  There appears to be no way to draw a line 
*          with an arbitrary pattern and width.  Also it appears that drawing
*          patterned lines with transparent background color is not supported.
*
*          Harold Youngren 10/01
***********************************************************************/
 

/***********************************************************************
* Defines graphics primitives for window management and line drawing
*  Primitives include:
*   gwxrevflag - checks environment variables for background color 
*   gwxopen    - initializes X display and returns size and depth of display
*   gwxwinopen - opens X plotting window with specified x,y size and position
*   gwxclear   - clears plotting window
*   gwxstatus  - gets current window size and location
*   gwxresize  - resizes current window to specified size 

*   gwxreset   - resets plotting defaults for window
*   gwxclose   - closes plotting to X display
*   gwxflush   - flushes out graphics primitives in buffers
*   gwxline    - plots line segment 

*   gwxdash    - sets line pattern from integer mask
*   gwxcurs    - gets graphics cursor position and key pressed
*   gwxpen     - sets line width in pixels
*
* More advanced routines beyond the original PLOT-10 requirements
*   gwxdestroy  - closes plot window
*   gwxlinez    - plots polyline
*   gwxpoly     - plots filled polygon
*   gwxstring  - plots string
*
*  Color routines 
*   gwxsetcolor      - sets foreground color from color map
*   gwxsetbgcolor    - sets background color from color map
*   gwxcolorname2rgb - find color components of color specified by name string
*   gwxallocrgbcolor - allocate a color specified by r,g,b components
*   gwxfreecolor     - frees an allocated color from colormap
*
* Utility routines 
*   mskbits    - converts integer mask into dot/dash array
*
* Double-buffer routines 
*   gwxdisplaybuffer - switches background buffer with foreground window
*   gwxdrawtobuffer  - sets drawing to background buffer
*   gwxdrawtowindow  - sets drawing to foreground window
************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <windows.h>
#include <process.h>

#include "rgbtbl.h"
#include "Xdefs.h"

#ifdef UNDERSCORE

#define MSKBITS          mskbits_
#define GWXREVFLAG       gwxrevflag_ 
#define GWXOPEN          gwxopen_ 
#define GWXWINOPEN       gwxwinopen_
#define GWXCLEAR         gwxclear_
#define GWXSTATUS        gwxstatus_
#define GWXRESIZE        gwxresize_
#define GWXRESET         gwxreset_
#define GWXCLOSE         gwxclose_
#define GWXFLUSH         gwxflush_
#define GWXLINE          gwxline_
#define GWXDASH          gwxdash_
#define GWXCURS          gwxcurs_
#define GWXCURSC         gwxcursc_
#define GWXPEN           gwxpen_
#define GWXDESTROY       gwxdestroy_
#define GWXLINEZ         gwxlinez_
#define GWXPOLY          gwxpoly_
#define GWXSTRING        gwxstring_
#define GWXSETCOLOR      gwxsetcolor_
#define GWXSETBGCOLOR    gwxsetbgcolor_
#define GWXCOLORNAME2RGB gwxcolorname2rgb_
#define GWXALLOCRGBCOLOR gwxallocrgbcolor_
#define GWXFREECOLOR     gwxfreecolor_
#define GWXDISPLAYBUFFER gwxdisplaybuffer_
#define GWXDRAWTOBUFFER  gwxdrawtobuffer_
#define GWXDRAWTOWINDOW  gwxdrawtowindow_

#else

#define MSKBITS          mskbits
#define GWXREVFLAG       gwxrevflag 
#define GWXOPEN          gwxopen 
#define GWXWINOPEN       gwxwinopen
#define GWXCLEAR         gwxclear
#define GWXSTATUS        gwxstatus
#define GWXRESIZE        gwxresize
#define GWXRESET         gwxreset
#define GWXCLOSE         gwxclose
#define GWXFLUSH         gwxflush
#define GWXLINE          gwxline
#define GWXDASH          gwxdash
#define GWXCURS          gwxcurs
#define GWXCURSC         gwxcursc
#define GWXPEN           gwxpen
#define GWXDESTROY       gwxdestroy
#define GWXLINEZ         gwxlinez
#define GWXPOLY          gwxpoly
#define GWXSTRING        gwxstring
#define GWXSETCOLOR      gwxsetcolor
#define GWXSETBGCOLOR    gwxsetbgcolor
#define GWXCOLORNAME2RGB gwxcolorname2rgb
#define GWXALLOCRGBCOLOR gwxallocrgbcolor
#define GWXFREECOLOR     gwxfreecolor
#define GWXDISPLAYBUFFER gwxdisplaybuffer
#define GWXDRAWTOBUFFER  gwxdrawtobuffer
#define GWXDRAWTOWINDOW  gwxdrawtowindow

#endif

typedef struct {
    HWND   window;
    HPEN   pen;
    HBRUSH fgbrush;
    HBRUSH bgbrush;
    int    fg;
    int    bg;
    int    font;
    int    fun;
} GC;  

typedef struct {
    int    ic;
    int    rgb;
    HPEN   pen;
    HBRUSH brush;
} COLTBL;  

typedef struct Event {
    HWND   window;
    int    type;
    int    x;
    int    y;
    int    state;
    struct Event *next;
} Event;  


/* Calling convention, Intel Fortran is simple, call is same as Unix
                       CVF requires Microsoft call  */
#ifdef _CVF
#define W32CALL __stdcall
#else
#define W32CALL
#endif


/* Maximum number of polyline points per polyline call, 
   increase if necessary */
#define MAXPTS 1000

/* Maximum number of colors */
#define MAXCOLS 256

static char *gwxClass = "PltLib";
static char *gwxName  = "PltLib";

/*
BYTE ANDMask[128], ORNMask[128], ORWMask[128];
static BYTE ANDMsk16[]     = { 0xfe, 0x00,
                               0xfe, 0x00,
                               0xfe, 0x00,
                               0xff, 0x80,
                               0xff, 0x80,
                               0xff, 0x00,
                               0xfe, 0x00,
                               0xfc, 0x18,
                               0xf8, 0x38,
                               0xf0, 0x7f,
                               0xe0, 0xff,
                               0xc1, 0xff,
                               0x83, 0xff,
                               0x07, 0xff,
                               0x0f, 0xff,
                               0x1f, 0xff };
    static BYTE ORNMsk16[] = { 0x00, 0x00,
                               0x00, 0xfe,
                               0x00, 0x7e,
                               0x00, 0x3e,
                               0x00, 0x3e,
                               0x00, 0x7e,
                               0x00, 0xe6,
                               0x01, 0xc2,
                               0x03, 0x80,
                               0x07, 0x00,
                               0x0e, 0x00,
                               0x1c, 0x00,
                               0x38, 0x00,
                               0x70, 0x00,
                               0x60, 0x00,
                               0x00, 0x00 };
    static BYTE ORWMsk16[] = { 0x01, 0xff,
                               0x01, 0x01,
                               0x01, 0x01,
                               0x00, 0x41,
                               0x00, 0x41,
                               0x00, 0x81,
                               0x01, 0x19,
                               0x02, 0x25,
                               0x04, 0x47,
                               0x08, 0x80,
                               0x11, 0x00,
                               0x22, 0x00,
                               0x44, 0x00,
                               0x88, 0x00,
                               0x90, 0x00,
                               0xe0, 0x00 };
*/

/* Window and graphics global data */
HWND     gwxWin;
HDC      gwxHDC, gwxmemHDC;
HBITMAP  gwxBMAP;
HCURSOR  gwxNormalC, gwxWaitC, gwxCurrentC;

GC       *gwxGC;

COLTBL   RGBmap[MAXCOLS];
int      nRGBmap;
  
/* int      xcsize,ycsize; */
int      wxstart,wystart,wxsize,wysize;
int      xrootsize, yrootsize, rootdepth;
int      gwxSize[5];
int      gwxXmatte, gwxYmatte, gwxYbar;
int      reversevideo;
int      initthread, initwnd;
int      penwidth, penpattern, ndash, pendash[16];

HINSTANCE  gwxInstance;
ATOM	   gwxRegister;
WNDCLASSEX gwxWndClass;
MSG        Message;

Event    *gwxEvents, *gwxLEvent, gwxLast;

/*static PARAMS params;*/

void MSKBITS(int*,int*,int*);
int gwxlookupcolor (char*,int*);



/**********************************************************************/
/************************Internal Event Code***************************/

/*These routines simulate the Xwindows XEvents to do user interaction.
  This is done here by collecting a list of relevant events from Windoze
  and managing these for the user who deals only with the simulated XEvents.
  This is probably a grungy hack (it was copied from Bob Haimes Visual3 and
  pV3 ports to Win32) but it works.  It may be better to get rid of the 
  extra event layer and deal directly with Win32 events.
  HHY */

void
gwxAddEvent(int type, int x, int y, int state)
{
    Event *event;

    event = (Event *) malloc(sizeof(Event));
    if (event == NULL) return;

    event->window = gwxWin;
    event->type   = type;
    event->x      = x;
    event->y      = y;
    event->state  = state;
    event->next   = NULL;

    if (gwxLEvent != NULL) gwxLEvent->next = event;
    gwxLEvent = event;
    if (gwxEvents == NULL) gwxEvents = event;
    gwxLast = *event;
}

void
gwxRemEvent(int *type, int *x, int *y, int *state)
{
    Event *event;

    if (gwxEvents == NULL) {
      printf("Warning: No Events to remove!\n");
      return;
    }

    event = gwxEvents;
    if (event == gwxLEvent) gwxLEvent = NULL;
    gwxEvents = event->next;

    *type  = event->type;
    *x     = event->x;
    *y     = event->y;
    *state = event->state;
    free(event);
}

/**********************************************************************/
/**********************************************************************/



LRESULT CALLBACK
gwxWndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    int         i, ix, iy;
    PAINTSTRUCT paint;
    HDC         hdc;
    POINT       point;
    RECT        rect;

    switch (message) {

    case WM_USER:
        return 0;

    case WM_SIZE:
          if (hWnd == gwxWin) {
            if (gwxSize[4]) {
              ix = LOWORD(lParam);
              iy = HIWORD(lParam);
              if ((ix != gwxSize[0]) || (iy != gwxSize[1])) {
                printf("WM_SIZE x=%d y=%d \n",gwxSize[0],gwxSize[1]); 
                printf("   new  x=%d y=%d \n",ix,iy); 
                gwxSize[0] = ix;
                gwxSize[1] = iy;
                gwxAddEvent(XExpose, ix, iy, 1);
              }
            } else {
              ix = gwxSize[0] + 2*gwxXmatte;
              iy = gwxSize[1] + 2*gwxYmatte + gwxYbar;
              SetWindowPos(hWnd, HWND_TOP, gwxSize[2], gwxSize[3],
                           ix, iy, SWP_NOMOVE|SWP_NOZORDER);
            }
            return 0;
            printf("WM_SIZE\n"); 
          }
        return 0;

    case WM_MOVE:
          if (hWnd == gwxWin) {
            gwxSize[2] = LOWORD(lParam);
            gwxSize[3] = HIWORD(lParam);
            return 0;
          }
        return 0;

    case WM_PAINT:
        printf("WM_PAINT\n"); 
        initwnd = 1;
        /*
         *  Validate the region even if there are no DisplayFunc.
         *  Otherwise, USER will not stop sending WM_PAINT messages.
         */
        hdc = BeginPaint(hWnd, &paint);
        BitBlt(hdc,0,0,xrootsize,yrootsize,gwxmemHDC,0,0,SRCCOPY);
        EndPaint(hWnd, &paint);
          if (hWnd == gwxWin) {
            gwxAddEvent(XExpose, gwxSize[0], gwxSize[1], 1);
            return 0;
          }
        return 0;

    case WM_PALETTECHANGED:
	printf("WM_PALETTECHANGED\n");
        return 0;

    case WM_ACTIVATE:
        printf("WM_ACTIVATE\n"); 
        break;

    case WM_MOUSEMOVE:
/* printf("mousemove active %d focus %d\n",GetActiveWindow(),GetFocus()); */
        if (GetActiveWindow() != hWnd) {
           SetForegroundWindow(hWnd);
           SetActiveWindow(hWnd);
        }
        if (GetFocus() != hWnd) SetFocus(hWnd);
/*	printf("mousemove %d  %d\n",hWnd,gwxWin);  */
        gwxAddEvent(XMotionNotify, LOWORD(lParam), HIWORD(lParam), 2);
        return 0;

    case WM_LBUTTONDOWN:
	if (GetAsyncKeyState(VK_RBUTTON) < 0) {
         gwxAddEvent(XButtonPress, LOWORD(lParam), HIWORD(lParam), 2);
	} else {
         gwxAddEvent(XButtonPress, LOWORD(lParam), HIWORD(lParam), 1);
	}
        return 0;

    case WM_LBUTTONUP:
	if (GetAsyncKeyState(VK_RBUTTON) < 0) {
	  gwxAddEvent(XButtonRelease, LOWORD(lParam), HIWORD(lParam), 2);
	} else {
	  gwxAddEvent(XButtonRelease, LOWORD(lParam), HIWORD(lParam), 1);
	}
        return 0;

    case WM_MBUTTONDOWN:
        gwxAddEvent(XButtonPress, LOWORD(lParam), HIWORD(lParam), 2);
        return 0;

    case WM_MBUTTONUP:
        gwxAddEvent(XButtonRelease, LOWORD(lParam), HIWORD(lParam), 2);
        return 0;

    case WM_RBUTTONDOWN:
	if (GetAsyncKeyState(VK_LBUTTON) < 0) {
	 gwxAddEvent(XButtonPress, LOWORD(lParam), HIWORD(lParam), 2);
	} else {
         gwxAddEvent(XButtonPress, LOWORD(lParam), HIWORD(lParam), 3);
	}
        return 0;

    case WM_RBUTTONUP:
	if (GetAsyncKeyState(VK_LBUTTON) < 0) {
	 gwxAddEvent(XButtonRelease, LOWORD(lParam), HIWORD(lParam), 2);
	} else {
         gwxAddEvent(XButtonRelease, LOWORD(lParam), HIWORD(lParam), 3);
	}
        return 0;

    case WM_KEYDOWN:
        GetCursorPos(&point);
        GetWindowRect(hWnd, &rect);
        ix = point.x - rect.left - gwxXmatte;
        iy = point.y - rect.top  - gwxYmatte - gwxYbar;
	/*        gwxAddEvent(XKeyPress, ix, iy, wParam+256); */
        return 0;

    case WM_KEYUP:
        GetCursorPos(&point);
        GetWindowRect(hWnd, &rect);
        ix = point.x - rect.left - gwxXmatte;
        iy = point.y - rect.top  - gwxYmatte - gwxYbar;
        i  = wParam+256;
	/*        if (i == 300) gwxAddEvent(XKeyPress, ix, iy, i);
        gwxAddEvent(XKeyRelease, ix, iy, i); */
        return 0;

    case WM_CHAR:
        if (wParam == 27) return 0;
        if ((gwxLast.type !=   2) || (gwxLast.state < 352) || 
            (gwxLast.state > 361) || (gwxLast.state-wParam != 304)) {
          GetCursorPos(&point);
          GetWindowRect(hWnd, &rect);
          ix = point.x - rect.left - gwxXmatte;
          iy = point.y - rect.top  - gwxYmatte - gwxYbar;
          gwxAddEvent(XKeyPress, ix, iy, wParam);
        } else {
          printf("Double Hit: state = %d\n", wParam);
        }
        return 0;

    case WM_SYSCOMMAND:
        /* special code for F10 */
        if ((lParam == 0) && (wParam == 61696)) {
          GetCursorPos(&point);
          GetWindowRect(hWnd, &rect);
          ix = point.x - rect.left - gwxXmatte;
          iy = point.y - rect.top  - gwxYmatte - gwxYbar;
          gwxAddEvent(XKeyPress, ix, iy, 377);
          return 0;
        }
        break;

    case WM_CLOSE:
        gwxAddEvent(XKeyPress, 0, 0, 283);
        exit;
        return 0;

    case WM_DESTROY:
        PostQuitMessage (0) ;
        return 0 ;
   }
    return DefWindowProc( hWnd, message, wParam, lParam);
}


int
rgb2winrgb(int acol)
{
    int    col, r, g, b;

    col = acol & 0x00ffffff;
    r   =  col >> 16;
    g   = (col >>  8) & 0xff;
    b   =  col        & 0xff;
    col = b << 16 | g << 8 | r;
    return col;
}



/* gwxrevflag
   Get XPLOT11 background default from users environment
   Parameters:  
     revflag          (int*)   reverse video flag (0 for white, 1 for black)
*/
void W32CALL 
GWXREVFLAG(revflag)
   int   *revflag;
{  
   char *bufp, *tmp;
/* check environment variable XPLOT11_BACKGROUND for background color
   XPLOT11_BACKGROUND  = white gives black on white plotting (like paper)
   XPLOT11_BACKGROUND != white gives white on black plotting (reverse video)
*/
   *revflag = 1;
   bufp = getenv("XPLOT11_BACKGROUND");

/* check lowercased environment for "black" to reverse video */
   if(bufp) {    
      for(tmp = bufp; *tmp; tmp++) 
         *tmp = tolower(*tmp); 
      *revflag = (strcmp(bufp,"white")!=0);
   }
}



/* gwxopen
   Open X window display and get size and depth of root window
   Parameters:  
     xsize,ysize            (int*)   root window size
     depth                  (int*)   screen color depth (pixel depth)
*/
void W32CALL 
GWXOPEN(int *xsize, int *ysize, int *depth)
{
	int i, j, k;

/* set depth arbitrarily to 8 bits to work like Xwindows colormaps */
    rootdepth = 8;
    xrootsize = GetSystemMetrics(SM_CXSCREEN);
    yrootsize = GetSystemMetrics(SM_CYSCREEN);

    *depth = rootdepth;
    *xsize = xrootsize;
    *ysize = yrootsize;

    gwxXmatte = GetSystemMetrics(SM_CXFRAME);
    gwxYmatte = GetSystemMetrics(SM_CYFRAME);
    gwxYbar   = GetSystemMetrics(SM_CYCAPTION) -
                GetSystemMetrics(SM_CYBORDER);
        
    printf("Screen is %d x %d\n", *xsize, *ysize);
    if (gwxXmatte == gwxYmatte) {
      printf("Window matte is %d\n",gwxXmatte);
    } else {
      printf("Window matte is %d x %d\n", gwxXmatte, gwxYmatte);
    }
    printf("Title bar height = %d\n", gwxYbar); 
    
 
/* check environment variables for default background color
   XPLOT11_BACKGROUND != black gives black on white plotting (like paper)
   XPLOT11_BACKGROUND  = black gives white on black plotting (reverse video) */
   GWXREVFLAG(&reversevideo); 

/* set up cursor bitmap arrays */
   /*	xcsize = GetSystemMetrics(SM_CXCURSOR);
	ycsize = GetSystemMetrics(SM_CYCURSOR);
	if ((xcsize != 16) && (xcsize != 32)) {
        printf("Unkown Xcursor Size %d\n", xcsize);
	    exit(1);
    }
	if ((ycsize != 16) && (ycsize != 32)) {
        printf("Unkown Ycursor Size %d\n", ycsize);
	    exit(1);
    }
	for (i = 0; i < 128; i++) {
        ANDMask[i] = 0xff;
		ORNMask[i] = 0;
		ORWMask[i] = 0;
	}
	j = 3;
	if (xcsize == 16) j = 1;
    for (k = i = 0; i < 16; i++) {
		ANDMask[k] = ANDMsk16[i*2];
	    ORNMask[k] = ORNMsk16[i*2];
		ORWMask[k] = ORWMsk16[i*2];
	    k++;
	    ANDMask[k] = ANDMsk16[i*2+1];
	    ORNMask[k] = ORNMsk16[i*2+1];
		ORWMask[k] = ORWMsk16[i*2+1];
	    k += j;
	}
	*/

/* set cursor */
    gwxWaitC   = LoadCursor(NULL, IDC_WAIT);
    gwxNormalC = LoadCursor(NULL, IDC_CROSS);
/*  gwxNormalC = CreateCursor( gwxInstance, 14, 1, xcsize, ycsize,
                               ANDMask, ORNMask);  */
/*  gwxInputC  = CreateCursor( gwxInstance, 14, 1, xcsize, ycsize,
                               ANDMask, ORWMask); */
    gwxCurrentC = gwxNormalC;


/* Define window class */
    gwxInstance = GetModuleHandle(NULL);
   
    gwxWndClass.cbSize        = sizeof(gwxWndClass);
    gwxWndClass.style         = CS_HREDRAW | CS_VREDRAW;
    gwxWndClass.lpfnWndProc   = (WNDPROC)gwxWndProc;
    gwxWndClass.cbClsExtra    = 0;
    gwxWndClass.cbWndExtra    = 0;
    gwxWndClass.hInstance     = gwxInstance;
    gwxWndClass.hIcon         = LoadIcon(NULL, IDI_APPLICATION);
    gwxWndClass.hCursor       = gwxNormalC;
    if(reversevideo) {
      gwxWndClass.hbrBackground = GetStockObject(BLACK_BRUSH); }
    else {
      gwxWndClass.hbrBackground = GetStockObject(WHITE_BRUSH); }
    gwxWndClass.lpszMenuName  = NULL;
    gwxWndClass.lpszClassName = (LPCSTR)gwxClass;
   
 /* Register the window class */
	gwxRegister = RegisterClassEx(&gwxWndClass);

 /* Check for window register, bomb if no joy... */
    if(gwxRegister == 0) {
        printf("Failed to register window class\n");
	    exit(1);
    }

/* initialize private Xevents loop pointers */
    gwxEvents   = NULL;
    gwxLEvent   = NULL;

}





GC* 
gwxgc(HWND *window, int *fontsize)
{
    GC *gc;
    int r, g, b, ic;

    gc = (GC *) malloc(sizeof(GC));

    gc->window = *window;
    if(reversevideo) {
      gc->fg     = RGB(255,255,255);
      gc->bg     = RGB(0  ,0  ,0  );
    }
    else {
      gc->fg     = RGB(0  ,0  ,0  );
      gc->bg     = RGB(255,255,255);
    }
    gc->pen      = CreatePen(PS_SOLID, penwidth, gc->fg);
    gc->fgbrush  = CreateSolidBrush(gc->fg);
    gc->bgbrush  = CreateSolidBrush(gc->bg);
    gc->font     = *fontsize;
    gc->fun      = GXcopy;                 /* copy */

    return gc;
}


void
gwxfreegc(GC **gc)
{
    GC *gcontext;

    gcontext = *gc;
    DeleteObject(gcontext->pen);
    DeleteObject(gcontext->fgbrush);
    DeleteObject(gcontext->bgbrush);
    free(*gc);
}


void 
gwxgcfun(int *fun)
{
    GC *gcontext;

    gcontext = gwxGC;
    gcontext->fun = *fun;
}



void
Thread1(PVOID pvoid)
{
    HWND winFocus, winFG, win;
//    HWND winF77;  
    RECT WinRect ,oldrect;
    POINT point;
    int  fontsize; 
    int  ix,iy; 
//    int  lunit; 

        printf("\nEntering Thread1 display thread\n"); 
    /*
     *  Make window large enough to hold a client area compensating for borders
     */
    WinRect.left   = wxstart;
    WinRect.top    = wystart;
    WinRect.right  = wxstart + wxsize + 2*gwxXmatte;
    WinRect.bottom = wystart + wysize + 2*gwxYmatte + gwxYbar; 
      /*    WinRect.right  = wxstart + wxsize;
	    WinRect.bottom = wystart + wysize; */

    
    printf("\nwxstart %d wystart %d \n",wxstart,wystart);
    printf(  "wxsize  %d wysize  %d \n",wxsize,wysize);

    printf("\nsetting rectangle left %d right %d \n",WinRect.left,WinRect.right);
    printf(  "setting rectangle top  %d bot   %d \n",WinRect.top,WinRect.bottom);
   
    
    winFG    = GetForegroundWindow();
    winFocus = GetActiveWindow();
    GetWindowRect(winFG, &oldrect);
    /*
    printf("Entry conditions:\n foreground %d/n orig focus %d\n",winFG,winFocus);
    printf("\nold rectangle left %d right %d \n",oldrect.left,oldrect.right);
    printf("\nold rectangle top  %d bot   %d \n",oldrect.top,oldrect.bottom);
    */

    /*    lunit = 5;
    winF77 = fgethwndqq(&lunit);
    printf("\nEntry conditions DVF window: %d/n",winF77);
    */ 

/* Must use WS_CLIPCHILDREN and WS_CLIPSIBLINGS styles. */

    gwxWin   = CreateWindow( gwxClass, gwxClass,
	       WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN | WS_CLIPSIBLINGS | WS_VISIBLE,
               WinRect.left, WinRect.top,
               WinRect.right - WinRect.left, WinRect.bottom - WinRect.top,
               NULL, NULL, gwxInstance, NULL );

    if (gwxWin == NULL)  {
        fprintf(stderr, "Error: NULL window handle.\n");
        exit(1);
    }

    ShowWindow(gwxWin, SW_SHOWDEFAULT);
    UpdateWindow(gwxWin);

    gwxHDC = GetDC(gwxWin);
    if (gwxHDC == NULL) {
        fprintf(stderr, "Error: NULL window DC.\n");
        exit(1);
    }

    /* Create compatible window bitmap */
    gwxmemHDC = CreateCompatibleDC(gwxHDC);
    gwxBMAP   = CreateCompatibleBitmap(gwxHDC,xrootsize,yrootsize);
    SelectObject(gwxmemHDC, gwxBMAP);

    SelectObject(gwxmemHDC, GetStockObject(ANSI_VAR_FONT));
    SetTextAlign(gwxmemHDC, TA_LEFT | TA_BASELINE | TA_NOUPDATECP);
    SetBkMode(gwxmemHDC, TRANSPARENT);
    
    gwxSize[0] = wxsize;
    gwxSize[1] = wysize;
    gwxSize[2] = wxstart;
    gwxSize[3] = wystart;
    gwxSize[4] = 1;
    printf("Thread1 xsize=%d ysize=%d \n",gwxSize[0],gwxSize[1]); 
    ix = gwxSize[0] + 2*gwxXmatte;
    iy = gwxSize[1] + 2*gwxYmatte + gwxYbar;
    SetWindowPos(gwxWin, HWND_TOP, gwxSize[2], gwxSize[3],
                 ix, iy, SWP_NOMOVE|SWP_NOZORDER);

    /* SetWindowPos(gwxWin, HWND_TOP, 0,0, 0,0, SWP_NOMOVE | SWP_NOSIZE ); */
    SetForegroundWindow(gwxWin); 


    nRGBmap     = 0;
    penwidth    = 0;
    penpattern  = -1;
    ndash    = 0;
    fontsize = 1;

    gwxGC = gwxgc(&gwxWin, &fontsize);
    SelectObject(gwxmemHDC, gwxGC->pen);

    /* initially clear the background bitmap */
    SelectObject(gwxmemHDC, gwxGC->bgbrush);
    PatBlt(gwxmemHDC, 0, 0, xrootsize, yrootsize, PATCOPY);

    SelectObject(gwxmemHDC, gwxGC->fgbrush);

    /* allocate black and white colors */
/*    r = 0;
      g = 0;
      b = 0;
      GWXALLOCRGBCOLOR(&r,&g,&b,&ic);
      r = 0;
      g = 0;
      b = 0;
      GWXALLOCRGBCOLOR(&r,&g,&b,&ic);
*/

    /*    printf("entering message loop\n\n"); */

    initthread = 1;

    /*    while (GetMessage(&Message, NULL, 0, 0)) { */

    /* Custom message loop to check cursor window to do X-style focus */

    while (1) {
      if(PeekMessage(&Message, gwxWin, 0, 0, PM_REMOVE)) {

	if(Message.message==WM_QUIT) 
          break;

       TranslateMessage(&Message);
       DispatchMessage(&Message);
       Sleep(1);
      }
      else {
       GetCursorPos(&point); 
       win = WindowFromPoint(point);
       /*       printf("pos x %d y %d win %d\n",point.x,point.y,win);  */

       /*       if(win==winF77) {
        SetActiveWindow(win);
        SetFocus(win);
        SetForegroundWindow(win); 
       } */
       /*       if(win == winFG) {
	 printf("pos x %d y %d win %d winFG %d\n",point.x,point.y,win,winFG); 
         SetActiveWindow(winFG);
         SetFocus(winFG);
         SetForegroundWindow(winFG);
       } */
       Sleep(1);
      } 
    }

    /*    printf("window thread ending\n"); */
/*  return Message.wParam;
    exit; */

    _endthread();
}


/* gwxwinopen
   Open window of specified size and position, return size and depth
   Parameters:  
     xstart,ystart          (int*)   upper left corner coordinates in root
     xsize,ysize            (int*)   desired window size
*/

void W32CALL 
GWXWINOPEN (int *xstart, int *ystart, int *xsize, int *ysize)
{
  
    printf("GWXOPEN\nxstart %d\nystart %d\nxsize %d\nysize %d\n",*xstart,*ystart,*xsize,*ysize);
    
    wxstart  = *xstart;
    wystart  = *ystart;
    wxsize   = *xsize;
    wysize   = *ysize;

    /* Start the window thread */
    initthread = 0;
    initwnd    = 0;
    _beginthread(Thread1,0,NULL);
    printf("beginthread called\n");

    while(!initthread || !initwnd) {
      Sleep(5);
    }

    /*    printf("GWXOPEN after thread start initthread %d initwnd %d\n",initthread, initwnd); */

}




void W32CALL
GWXDESTROY()
{
    ReleaseDC( gwxWin, gwxHDC );
    ReleaseDC( gwxWin, gwxmemHDC );
}


void W32CALL
GWXCLOSE()
{
    PostQuitMessage (0) ;
    /*    DestroyCursor(gwxNormalC);
          DestroyCursor(gwxInputC); */
    UnregisterClass((LPCSTR)gwxClass, gwxInstance);
}


void W32CALL
GWXFLUSH()
{  
    BitBlt(gwxHDC,0,0,xrootsize,yrootsize,gwxmemHDC,0,0,SRCCOPY);
    /*    InvalidateRect(gwxWin,NULL,1); */
}

 
/* gwxreset
   Reset graphics context to default
   Parameters:   None
*/
void W32CALL
GWXRESET()
{
}


/* getPEN
   Creates pen using type, color, width, pattern
*/
HPEN
makenewpen()
{
    HPEN newpen;
    LOGBRUSH lb;
    GC  *gcontext;

    gcontext = gwxGC;

    /*  printf("makenewpen\n ndash %d\n lmask %d\n fg %d\n width %d\n",ndash,penpattern,gcontext->fg,penwidth); */

    if(ndash == 0) {
      if(penpattern == 0) {
	/*        printf("making penpattern 0\n"); */
        newpen = CreatePen(PS_SOLID, penwidth, gcontext->bg);
      }
      if (penpattern == -1) {
	/*        printf("making penpattern -1\n"); */
        newpen = CreatePen(PS_SOLID, penwidth, gcontext->fg);
      }
    }
    else {
      /*      printf("making dash pen\n"); */
      newpen = CreatePen(PS_DOT, 0, gcontext->fg); 
/*
      lb.lbStyle = BS_SOLID;
      lb.lbColor = gcontext->fg;
      lb.lbHatch   = 0;
      newpen = ExtCreatePen(PS_GEOMETRIC | PS_USERSTYLE, 
                            penwidth, &lb, ndash, pendash);
			    */
    }
    return newpen;
}

int
GWXNUMEVENTS()
{
    int stat;

    stat = 1;
    if (gwxEvents == NULL) stat = 0;
    return stat;
}


void 
GWXEVENT(int *type, int *x, int *y, int *state)
{
    gwxRemEvent(type, x, y, state);
}


int 
GWXGETEVENT(int *type, int *x, int *y, int *state)
{
    int   stat;
    Event *event;

    stat  = -1;
    event = gwxEvents;
    gwxLEvent = NULL;
    while (event != NULL) {
      if ((*type == event->type) && (stat == -1)) {
        *x     = event->x;
        *y     = event->y;
        *state = event->state;
        stat   = 0;
        if (gwxLEvent == NULL) {
          gwxEvents = event->next;
        } else {
          gwxLEvent->next = event->next;
        }
        free(event);
        event = NULL;
        if (gwxLEvent != NULL) event = gwxLEvent->next;
      } else {
        gwxLEvent = event;
        event = event->next;
      }
    }
    return stat;
}






/* gwxstatus
   Return current window status (position, size)
   Parameters:  
     xstart,ystart          (int*)   upper left corner coordinates in root
     xsize,ysize            (int*)   desired window size
*/
void W32CALL
GWXSTATUS(int *xstart, int *ystart, int *xsize, int *ysize) 
          
{
    RECT  rect;

   
    GetClientRect(gwxWin, &rect);
    printf("\nstatus client rectangle left %d right %d \n",rect.left,rect.right);
    printf(  "                        top  %d bot   %d \n",rect.top,rect.bottom);
    
    GetWindowRect(gwxWin, &rect);
    *xstart = rect.left;
    *ystart = rect.top;
    *xsize = rect.right  - rect.left - 2*gwxXmatte;
    *ysize = rect.bottom - rect.top  - 2*gwxYmatte - gwxYbar;

    
    printf("\nstatus xstart %d ystart %d \n",*xstart,*ystart);
    printf(  "       xsize  %d ysize  %d \n",*xsize,*ysize);
    printf("\nstatus rectangle left %d right %d \n",rect.left,rect.right);
    printf(  "                 top  %d bot   %d \n",rect.top,rect.bottom);
    
}



/* gwxresize
   Resize screen window to x width, y height
   Parameters:  
     x           (int*)   new width
     y           (int*)   new height
*/
void W32CALL
GWXRESIZE(x, y)
   int     *x, *y;
{
   int ix, iy;

    gwxSize[0] = *x;
    gwxSize[1] = *y;
    ix = gwxSize[0] + 2*gwxXmatte;
    iy = gwxSize[1] + 2*gwxYmatte + gwxYbar;
    printf("\nresize x %d y %d \n",*x,*y);
    SetWindowPos(gwxWin, HWND_TOP, gwxSize[2], gwxSize[3],
                           ix, iy, SWP_NOMOVE|SWP_NOZORDER);
}


/* gwxclear
   Clear current plot window
   Parameters:  None
*/
void W32CALL
GWXCLEAR()
{
    int  i;
    RECT WinRect;

    WinRect.left   = 0;
    WinRect.right  = gwxSize[0] + 2*gwxXmatte;
    WinRect.top    = 0;
    WinRect.bottom = gwxSize[1] + 2*gwxYmatte + gwxYbar;
    FillRect(gwxmemHDC, &WinRect, gwxGC->bgbrush);
    return;
}


/* gwxdisplaybuffer
   Switches background buffer onto foreground window, displaying accumulated
   graphics
   Parameters:  None
*/
void W32CALL
GWXDISPLAYBUFFER()
{
}


/* gwxdrawtobuffer
   Switches graphics to draw to the background buffer
   Parameters:  None
*/
void W32CALL
GWXDRAWTOBUFFER()
{
/* printf("called gwxdrawtobuffer\n"); */
}

/* gwxdrawtowindow
   Switches graphics to draw to the foreground window
   Parameters:  None
*/
void W32CALL
GWXDRAWTOWINDOW()
{
/* printf("called gwxdrawtowindow\n"); */ 
}


void W32CALL
GWXSTRING(int *x, int *y, char *text, int *length)
{
    int i, xs, ys;
    GC  *gcontext;

    xs = *x + gwxXmatte;
    ys = *y + gwxYmatte;

    gcontext = gwxGC;
    if (gcontext->font != 3) {
      SelectObject(gwxmemHDC, GetStockObject(ANSI_VAR_FONT));
    } else {
      SelectObject(gwxmemHDC, GetStockObject(SYSTEM_FONT));
    }
    SetTextColor(gwxmemHDC, gcontext->fg);
    SetBkColor  (gwxmemHDC, TRANSPARENT);
    TextOut(gwxmemHDC, xs, ys, text, *length);
}


/* gwxline
   Draw line from x1,y1 to x2,y2
   Parameters:  
     x1,y1            (int*)   starting position for line
     x2,y2            (int*)   ending position for line
*/
void W32CALL
GWXLINE(int *x1, int *y1, int *x2, int *y2)
{
    int  i, xs, ys;

    xs = *x1;
    ys = *y1;
    MoveToEx(gwxmemHDC, xs, ys, NULL);
    xs = *x2;
    ys = *y2;
    LineTo(gwxmemHDC, xs, ys);
}


/* gwxlinez
   Draw polyline on window in current color and pen
   Parameters:  
     ix           (int*)   array of x coordinates on polyline
     iy           (int*)   array of y coordinates on polyline
     n            (int*)   number of coordinate points
    (see define for MAXPTS at start of this file)
*/
void W32CALL
GWXLINEZ(int *ix, int *iy, int *n)
{
    int  i, xs, ys;

    POINT points[MAXPTS];

   if (*n > MAXPTS) {
    fprintf(stderr,"Xplot11.gwxlinez: Too many points in polyline\n");
    printf("Xplot11.gwxlinez: Too many points in polyline\n");
    return;
   }

   for(i=0; i < *n; i++) {
      points[i].x = ix[i];
      points[i].y = iy[i];
/*    printf("gwxlinez x %d y %d \n",ix[i],iy[i]); */
    }
    Polyline(gwxmemHDC, points, *n);
}


/* gwxpoly
   Draw filled polyline on window in current color and pen
   Parameters:  
     x_coord      (int*)   array of x coordinates on polyline
     y_coord      (int*)   array of y coordinates on polyline
     n_coord      (int*)   number of coordinate points
    (see define for MAXPTS at start of this file)
*/
void W32CALL
GWXPOLY(x_coord, y_coord, n_coord)
   int     *x_coord, *y_coord, *n_coord;

{  POINT  points[MAXPTS];
   int  i,n;

   if (*n_coord >= MAXPTS) {
      fprintf(stderr,"Xplot11.gwxpoly: Too many points in polyline\n");
      printf("Xplot11.gwxpoly: Too many points in polyline\n");
      return;
    }

   for (i=0; i < *n_coord; i++) {
       points[i].x = x_coord[i];
       points[i].y = y_coord[i]; 

     }
   n = *n_coord;
/* if the polyline is not closed, duplicate first point to ensure closed 

   perimeter (speeds up X graphics drastically) */
   if ( (points[n-1].x != points[0].x) || 
        (points[n-1].y != points[0].y) ) {
       points[n].x = points[0].x;
       points[n].y = points[0].y;
       n++;
      }
    Polygon(gwxmemHDC, points, n);
}


void W32CALL
GWXARC(int *xc, int *yc,
        int *xr, int *yr, float *alpha, float *beta)
{
    int  i, j, xs, ys;

    /* only does circles! */

    xs = *xc + *xr;
    ys = *yc;
    MoveToEx(gwxmemHDC, xs, ys, NULL);
    for (j = 2; j <= 360; j += 2) {
      xs = (int) ((double)*xc + (double)*xr*cos((double)j*0.017453292));
      ys = (int) ((double)*yc + (double)*yr*sin((double)j*0.017453292));
      LineTo(gwxmemHDC, xs, ys);
    }
}

void W32CALL
GWXCURS(int *x, int *y, int *state)
{
int type;

/* Eat up all the current events first */

   while(GWXNUMEVENTS()) {
     GWXEVENT(&type, x, y, state);
     /*     printf("eat up event %d\n",type); */
   }
/* Now run an event loop until a buttonpress or keypress is found */

  while(1) {
    if(GWXNUMEVENTS()) { 
      GWXEVENT(&type, x, y, state);
      /*      printf("get event %d\n",type); */
      switch (type) {
       case XKeyPress:
         return;

       case XButtonPress:
         return;
      }
    }
  }
}

void W32CALL
GWXCURSC(int *x, int *y, int *state)
{
int type;

/* Eat up all the current events first */

   while(GWXNUMEVENTS()) {
     GWXEVENT(&type, x, y, state);
     /*     printf("eat up event %d\n",type); */
   }
/* Now run an event loop until a buttonpress or keypress is found */

  while(1) {
    if(GWXNUMEVENTS()) { 
      GWXEVENT(&type, x, y, state);
      /*      printf("get event %d\n",type); */
      switch (type) {
       case XMotionNotify:
         *state = 1;
	 if((GetAsyncKeyState(VK_LBUTTON) < 0) || 
            (GetAsyncKeyState(VK_MBUTTON) < 0) ||
            (GetAsyncKeyState(VK_RBUTTON) < 0)) *state = 1;
         return;

       case XButtonRelease:
         *state = 0;
         return;
      }
    }
  }
}

 
void W32CALL
GWXCURS2(int *x, int *y, int *state)
{
  int   i, key;
  POINT point;

  key = 0;
  if  (GetAsyncKeyState(VK_SHIFT)   < 0)  key += 1;
  if  (GetAsyncKeyState(VK_CONTROL) < 0)  key += 4;
  if  (GetAsyncKeyState(VK_MENU)    < 0)  key += 8;
  if  (GetAsyncKeyState(VK_LBUTTON) < 0)  key += 256;
  if  (GetAsyncKeyState(VK_MBUTTON) < 0)  key += 512;
  if  (GetAsyncKeyState(VK_RBUTTON) < 0)  key += 1024;
  if ((GetAsyncKeyState(VK_LBUTTON) < 0) &&
      (GetAsyncKeyState(VK_RBUTTON) < 0)) key -= 768;

  GetCursorPos(&point);

  *x     = -1;
  *y     = -1;
  *state =  0;
  *x     = point.x - gwxSize[2];
  *y     = point.y - gwxSize[3];
  *state = key;
}


HWND W32CALL
GWXCURRENTPOINTER(int *x, int *y)
{
    POINT point;
    RECT  rect;
    HWND  win;

    GetCursorPos(&point);
    win = WindowFromPoint(point);
    GetWindowRect(win, &rect);
    *x = point.x - rect.left;
    *y = point.y - rect.top;
    return win;
}


void W32CALL
GWXSETPOINTER(HWND win, int *x, int *y)
{
    int   ix, iy; 
    RECT  rect;

    if (GetActiveWindow() != win) SetActiveWindow(win);
    if (GetFocus() != win) SetFocus(win);
    if (GetForegroundWindow() != win) SetForegroundWindow(win);
    GetWindowRect(win, &rect);
    ix = *x + rect.left;
    iy = *y + rect.top;
    SetCursorPos(ix, iy);
}




/* gwxdash
   Set line drawing pattern to mask pattern
   Parameters:  
     mask     (int*)  integer mask value (bits set pen pattern for lines)
*/
void W32CALL
GWXDASH(int* lmask)
{
GC *gcontext;
int   i, ndsh;
HPEN  oldpen;

    penpattern = *lmask;
    gcontext = gwxGC;

    if(*lmask==0) {
      ndash = 0;
    }
    else if(*lmask==-1) {
      ndash = 0;
    }
    else { 
      (void) MSKBITS(lmask,pendash,&ndsh);
      ndash = ndsh;
/*      for(i=1; i<=ndash; i++) printf("%d dashes %d\n",i,pendash[i-1]);  */
    }
    oldpen = gcontext->pen;
    gcontext->pen = makenewpen();
    oldpen = SelectObject(gwxmemHDC, gcontext->pen);
    DeleteObject(oldpen);
}

 

/* gwxpen
   Set pen width for line drawing 
   Parameters:  
     ipen     (int*)  integer pen width
*/
void W32CALL
GWXPEN(int* ipen)
{
GC *gcontext;
HPEN oldpen;

    penwidth = *ipen;
    gcontext = gwxGC;
    oldpen = gcontext->pen;
    gcontext->pen = makenewpen();
    oldpen = SelectObject(gwxmemHDC, gcontext->pen);
    DeleteObject(oldpen);
}


/* mskbits
   Utility routine to convert lower 16 bits of pattern mask
    into pattern array of on/off bit lengths.  The 16
    bits of ipat can contain up to 16 on/off bit lengths.
    i.e. 0XAAAAAAA produces ibits=(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1)
    i.e. 0XCCCCCCC produces ibits=(2,2,2,2,2,2,2,2)
    The output pattern stored in integer array ibits=(#on,#off,#on,#off,etc.)
   Parameters:  
     mask           (int*)   input mask
     ibits          (int*)   output pattern string (#on, #off...)
     ndash          (int*)   number of entries in pattern array
*/
void 
MSKBITS(int* mask, int* ibits, int* ndash)
{
#define BITSINMASK 16       /* use only lower 16 bits of mask word */

int            i,ic,ibit,ibitold;
int            nbits, nshft;
unsigned short lmask;

/* shift mask until low bit is 1, filling high bits with 0's */
	  lmask = *mask;
          nshft = 0;

	  if(lmask!=0) { 
 	    while (!(ibitold = (lmask & 0x01)))
              lmask >>= 1;
              nshft++;
	  }
/* if no 1 bits just exit with no dashes set */
          if(ibitold==0) {
	    *ndash = 0;
	    exit;
	  }

/*  cycle through the lower 'length-nshft' bits 
    checking for number of contiguous same bits, 
    store into ibits array  */
    	  nbits = ic = 0; 

	  for (i=0; i<(BITSINMASK-nshft); ++i) {

	    ibit=(lmask & 0x01);

	    if(ibit != ibitold) {
	      ibits[ic++] = nbits;
	      nbits = 0;
	     }

            ibitold = ibit; 
            nbits++; 

	    lmask >>= 1;
	  }

/* add bits at end of shifted bits including any initial shift 
   to find the first 1 in the low bit */
	  if(ibit==1) {

            ibits[ic++] = nbits;
            if(nshft>0) 
              ibits[ic++] = nshft;

	    }
           else
    	    ibits[ic++] = nbits + nshft;

	  *ndash = ic;
}      




/* gwxsetcolor
   Set foreground color to stored colormap pixel value
   Parameters:  
     icol     (int*)   index of pixel value (mapped in colormap)
*/
void W32CALL
GWXSETCOLOR(icol)
   int *icol;
{  
   int ic, n, fg;
    GC *gcontext;
  HPEN oldpen;

/* is the color index in range for stored colormap data? */
    ic = *icol;
/* find the color in the table */
    for (n=0; n < nRGBmap; n++) {
      if(ic == RGBmap[n].ic) {
        fg = RGBmap[n].rgb;
        gcontext = gwxGC;
        gcontext->fg  = rgb2winrgb(fg);
/*    printf("gwxsetfgcolor fg %d  wfg %d\n",*fg,gcontext->fg); */
        gcontext->pen = makenewpen(); 
        gcontext->fgbrush = RGBmap[n].brush;
        oldpen = SelectObject(gwxmemHDC, gcontext->pen);
        DeleteObject(oldpen);
        SelectObject(gwxmemHDC, gcontext->fgbrush);
        return;
      }
    }
    fprintf(stderr,"Xplot11.gwxsetcolor: color index %d out of range\n",*icol);

    printf("Xplot11.gwxsetcolor: color index %d out of range\n",*icol);
}



/* gwxsetbgcolor
   Set background color to pixel value
   Parameters:  
     pixel     (int*)   pixel value (mapped in colormap)
*/
void W32CALL
GWXSETBGCOLOR(icol)
   int *icol;
{  
   int ic, n, bg;
    GC *gcontext;
/* is the color index in range for stored colormap data? */
    ic = *icol;
/* find the color in the table */
    for (n=0; n < nRGBmap; n++) {
      if(ic == RGBmap[n].ic) {
        bg = RGBmap[n].rgb;
        gcontext = gwxGC;
        gcontext->bg  = rgb2winrgb(bg);
        gcontext->bgbrush = RGBmap[n].brush;
        return;
      }
    }
    fprintf(stderr,"Xplot11.gwxsetbgcolor: color index %d out of range\n",*icol);
    printf("Xplot11.gwxsetbgcolor: color index %d out of range\n",*icol);
}



/* gwxcolorname2rgb
   Find r,g,b components for color specified by name string 
   Parameters:  
     red,grn,blu   (int*)  output color components (0-255)
     nc            (int*)  string length (# of chars)
     colorname     (char*) string containing name of valid color
                           Note: color names are not case sensitive
     len           (int)   fortran appended string length (passed by value)

   Valid color names are any color name that is known to X11 color database
   Examples are "Black","White","Yellow","Orange","Red","Green",
                "Cyan","Blue","Magenta", "ivory", etc.
*/
void W32CALL
GWXCOLORNAME2RGB(int *red, int *grn, int *blu, 
                 int *nc, char *colorname, int len)
{
  char cname[32];
  int i,n;
  int irgb;

  n = *nc;
  /* copy string to avoid overwriting possibly static input string
     Note that string length is explicitly passed to avoid compatibility
     problems with by fortran character arg length */
  if(n>31)  {
    n = 31;
    fprintf(stderr,"Xplot11.gwxcolorname2rgb: color name '%s' truncated\n",
	    cname);
    printf("Xplot11.gwxcolorname2rgb: color name '%s' truncated\n",cname);
  }
  strncpy(cname,colorname,n);
  cname[n] = '\0';

  *red = -1;
  *grn = -1;
  *blu = -1;
  if (!gwxlookupcolor(cname,&irgb)) {
    fprintf(stderr,"Xplot11.gwxcolorname2rgb: color name '%s' not found\n",
	    cname);
    printf("Xplot11.gwxcolorname2rgb: color name '%s' not found\n",cname);
    return;
  }
  *red = 0x000000ff & (irgb >> 16);
  *grn = 0x000000ff & (irgb >> 8); 
  *blu = 0x000000ff & (irgb); 

  /*  
   printf("gwxcolorname2rgb red = %d\ngrn = %d\nblu = %d\n",*red,*grn,*blu);
  */
}


int
gwxlookupcolor (char *colorname, int *irgb)
{
  int IC,ic,ncolors;
  int i;
  char cname[32], *ctmp;

  IC = sizeof(Colordef);
  ic = sizeof(colordef);
  ncolors = ic/IC;

  /*
  printf("Colordef %d\n",IC);
  printf("colordef %d\n",ic); 
  printf("#entries %d\n",ncolors);
  */

  if(ctmp=strcpy(cname,colorname)) {    
    for(ctmp = cname; *ctmp; ctmp++) 
       *ctmp = tolower(*ctmp);
  }
  /*  printf("cname %s\n",cname); */
  
  for (i=0; i < ncolors; i++) {
   if(!strcmp(colordef[i].name,cname)) {
   /* printf("found string %s code %d\n",colordef[i].name,colordef[i].rgb); */
    *irgb = colordef[i].rgb;
    return(1);
   }
  }
  return(0);
}


/* gwxallocrgbcolor
   Allocate a color in colormap specified by r,g,b components 

   Parameters:  
     red,grn,blu   (int*)  input color components (0-255)
     ic            (int*)  returned color index or pixel value
*/
void W32CALL
GWXALLOCRGBCOLOR(int *red, int *grn, int *blu, int *ic)
{
  int  rgb,col;
  int i;

  rgb = ((0xff & *red) << 16) | ((0xff & *grn) << 8) | (0xff & *blu);
  *ic = -1;

  /*  printf("red %d\ngrn %d\nblu %d\nrgb %d\n",*red,*grn,*blu,rgb); */

/* check for rgb color already allocated in table */

  if(nRGBmap > 0) {
    for (i=0; i < nRGBmap; i++) {
      if(rgb == RGBmap[i].rgb) {
        *ic = RGBmap[i].ic;
       /*       printf("found color %d in table at %d index %d\n",rgb,i+1,*ic);  */
        return;
      }
    }
  }
/* this color's RGB was not in table, allocate it to table
   if there is room allocate a new color */  

  if(nRGBmap < MAXCOLS) {
    RGBmap[nRGBmap].rgb = rgb;
    RGBmap[nRGBmap].ic  = nRGBmap+1;
    col = rgb2winrgb(rgb);
    RGBmap[nRGBmap].pen   = NULL;
    RGBmap[nRGBmap].brush = CreateSolidBrush(col);
    nRGBmap++;
    *ic = nRGBmap;
    /*    printf("allocating color table entry %d at %d\n",*ic,nRGBmap-1); */
  }
  else {
    fprintf (stderr,"Xplot11.gwxallocrgbcolor: can't allocate color.\n");
    printf ("Xplot11.gwxallocrgbcolor: can't allocate color.\n");
  }
}


/* gwxfreecolor
   Free a color from color map
   Parameters:  
     pix (int*)  pixel in colormap
*/
void W32CALL  
GWXFREECOLOR(int *icol)
{
  int i,ic,n;

/* is the color index in range for stored colormap data? */
    ic = *icol;
/*    printf("color to delete %d\n",ic);  */
    if((ic > 0) && (ic <= MAXCOLS)) {

/* find the color in the table */
/*      printf("checking colortable of size %d\n",nRGBmap); */
      for (n=0; n < nRGBmap; n++) {
/*        printf("comparing RGBmap %d with %d to %d\n",n,RGBmap[n].ic,ic); */
        if(ic == RGBmap[n].ic) {
/* delete the table entry */
/*          printf("found color %d to delete at %d\n",ic,n); */

          if(RGBmap[n].pen)   DeleteObject(RGBmap[n].pen);
          if(RGBmap[n].brush) DeleteObject(RGBmap[n].brush);

          if(n < nRGBmap-1) {
           for (i=n; i < nRGBmap; i++) {
            RGBmap[i-1].rgb = RGBmap[i].rgb;
            RGBmap[i-1].ic  = RGBmap[i].ic;
            RGBmap[i-1].pen   = RGBmap[i].pen;
            RGBmap[i-1].brush = RGBmap[i].brush;
           }
          }
          nRGBmap--;
/*          printf("deleted color %d\n",*icol);  */
          return;
	}
      }
    }
    fprintf(stderr,"Xplot11.gwxfreecolor: color index %d out of range\n",*icol);
    printf("Xplot11.gwxfreecolor: color index %d out of range\n",*icol);
}













