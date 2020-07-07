/*********************************************************************** 
    Module:  Xwin.c
 
    Copyright (C) 1996 Harold Youngren, Mark Drela 
 
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

    Report problems to:    guppy@maine.com 
                        or drela@mit.edu  
***********************************************************************/ 

/***********************************************************************
*  Xplot11 C-level X11 interface
C     Version 4.46 11/28/01
*
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
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/cursorfont.h>


/* Handle various system requirements for trailing underscores, or other 
   fortran-to-C interface shenanigans thru defines for routine names  
   The provided set gives the option of setting a compile flag -DUNDERSCORE
   to include underscores on C routine name symbols */

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


/* Maximum number of polyline points per polyline call, 
   increase if necessary */
#define MAXPTS 1000

#define argc 0
#define argv (char **) NULL

Display      *display;
Window        window;
Pixmap        pixmap;
Drawable      buffer;
Cursor        cursor;
GC            gc;
unsigned long fgcolor, bgcolor;
int           height, width;
int           line_width;
int 	      root_width, root_height, root_depth;
int           reversevideo;
static Window parent_window;
Colormap      cmap;

void MSKBITS(int*,int*,int*);


/**********************************************************************/




/* gwxrevflag
   Get XPLOT11 background default from users environment
   Parameters:  
     revflag          (int*)   reverse video flag (0 for white, 1 for black)
*/

void
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

/* check lowercased environment for "white" for non-reverse video */
   if(bufp) {    
      for(tmp = bufp; *tmp; tmp++) 
         *tmp = tolower(*tmp); 
      *revflag = (strcmp(bufp,"white")!=0);
   }
}


/* gwxopen
   Open X window display and get size and depth of root window
   Parameters:  
     xsize,ysize            (int*)   desired window size
     idepth                 (int*)   screen color depth (pixel depth)
     revflag                (int*)   reverse video flag
*/

void
GWXOPEN(xsizeroot, ysizeroot, depth)
   int   *xsizeroot, *ysizeroot, *depth;
{  

   int   revert,i;

/* check environment variables for default background color
   XPLOT11_BACKGROUND != black gives black on white plotting (like paper)
   XPLOT11_BACKGROUND  = black gives white on black plotting (reverse video) */
   GWXREVFLAG(&reversevideo);


/* open the display */
   display = XOpenDisplay(NULL);
   /* XSynchronize(display,1); */
   if (display == NULL) 

      {printf(" Cannot open display...aborting\n");
       exit(1);
      }

/* get old window focus to use later for cursor positioning */
   XGetInputFocus(display,&parent_window,&revert);
     

/* get root window size and depth attributes */
   root_width  = DisplayWidth(display, DefaultScreen(display));
   root_height = DisplayHeight(display, DefaultScreen(display));
   root_depth  = DefaultDepth(display, DefaultScreen(display));

   *xsizeroot = root_width; 
   *ysizeroot = root_height; 
   *depth = root_depth; 
}



/* gwxwinopen
   Open window of specified size and position, return size and depth
   Parameters:  
     xstart,ystart          (int*)   upper left corner coordinates in root
     xsize,ysize            (int*)   desired window size
*/

void 
GWXWINOPEN(xstart, ystart, xsize, ysize)
   int   *xstart, *ystart, *xsize, *ysize;
{
   XSizeHints           hints;
   XSetWindowAttributes wattrib;
   unsigned long        wattrib_mask;
   int 			x,y,win_x,win_y;
   unsigned long	pixel_white,pixel_black;
   XEvent               event;
   XColor               fgcurs, bgcurs;
   Font                 font;
   XGCValues            gcv;
   unsigned long        gcv_mask;

   char *fontname = "6x12";
 

/* Define a crosshair cursor */
   Pixmap curs;
   unsigned int curs_width=16;
   unsigned int curs_height=16;
   unsigned int curs_x_hot=7;
   unsigned int curs_y_hot=8;   /* Was: curs_y_hot=7   MD 3/12/96 */
   static unsigned char curs_bits[] =
   { 0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,
     0x80,0x00,0x80,0x00,0x80,0x00,0xff,0x7f,
     0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,
     0x80,0x00,0x80,0x00,0x80,0x00,0x00,0x00 };

/* check for open display */
   if (display == NULL) 

      {printf(" Cannot open display...aborting in gwxwinopen\n");
       exit(1);
      }

   width  = *xsize;
   height = *ysize;
/* printf("gwxwinopen size x %d  y %d \n",width,height);  */ 

/* set up window attributes and create window */
   pixel_white = WhitePixel(display, DefaultScreen(display));
   pixel_black = BlackPixel(display, DefaultScreen(display));
   if(reversevideo) {
     fgcolor = pixel_white;
     bgcolor = pixel_black; 
     }
    else {
     fgcolor = pixel_black;
     bgcolor = pixel_white; 
     }
 

/* define a colormap and window attributes*/
   cmap = DefaultColormap(display, DefaultScreen(display));  


   wattrib.colormap         = cmap;
   wattrib.background_pixel = bgcolor;
   wattrib.border_pixel     = fgcolor;
   wattrib.event_mask       = ExposureMask | ConfigureNotify | 

                              KeyPressMask | ButtonPressMask;
   wattrib_mask             = CWColormap   | CWEventMask     | 

                              CWBackPixel  | CWBorderPixel;

/*  Use backing store (if Expose events are to regen the display, 

    comment out these references to backing store  */
   wattrib.backing_store = WhenMapped;
   wattrib.backing_planes = AllPlanes;   

   wattrib.bit_gravity = SouthWestGravity;   

   wattrib.win_gravity = NorthWestGravity;   

   wattrib_mask        = wattrib_mask | 

                         CWBitGravity | CWBackingStore  |
                         CWWinGravity | CWBackingPlanes;     


/* create the window using these parameters */
   window = XCreateWindow(display, DefaultRootWindow(display),
                          *xstart, *ystart, width, height, 2, root_depth,
                          InputOutput, CopyFromParent,
                          wattrib_mask, &wattrib);
 
/* Set pixmap initally pointing to NULL to indicate it is yet not created
   to do double buffering */
   pixmap = (Pixmap) NULL;

/* set initial drawing destination to foreground window */
   buffer = window;
 

/* set up interaction with the window manager */
   hints.flags  = USPosition | USSize;
   hints.width  = width;
   hints.height = height;
   hints.x      = *xstart;
   hints.y      = *ystart;
   XSetNormalHints(display, window, &hints);
   XSetStandardProperties(display, window, "Xplot11\0", "Xplot11\0",
                          None, argv, argc, &hints);

/* set up cursor */
   fgcurs.pixel = fgcolor;
   bgcurs.pixel = bgcolor;
   XQueryColor(display,cmap,&fgcurs);
   XQueryColor(display,cmap,&bgcurs);

   if((curs=XCreateBitmapFromData(display,window,(char*) curs_bits,
                              curs_width,curs_height))!= None)
	{   cursor = XCreatePixmapCursor(display,curs,curs,
	             &fgcurs,&bgcurs,curs_x_hot,curs_y_hot);  }
	else
	{   cursor = XCreateFontCursor(display,XC_draft_small);  };

   XDefineCursor(display, window, cursor);
   XRecolorCursor(display, cursor, &fgcurs, &bgcurs); 


/* create the gc */
   line_width = 0;
   gcv.function   = GXcopy;
   gcv.foreground = fgcolor;
   gcv.background = bgcolor;
   gcv.line_width = line_width;
   gcv.line_style = LineSolid;
   gcv.cap_style  = CapButt;
   gcv.fill_style = FillSolid;
   gcv.join_style = JoinMiter; 

   gcv_mask = GCFunction  | GCForeground | GCBackground | GCLineWidth | 

              GCLineStyle | GCCapStyle   | GCFillStyle  | GCJoinStyle; 

   gc = XCreateGC(display, window, gcv_mask, &gcv);

/* load the font */
   font = XLoadFont(display, fontname);
   XSetFont(display, gc, font);

/* map the window and wait for an expose event to proceed */
   XMapWindow(display, window);
   while(1)  
   { XNextEvent(display, &event);
     if(event.type==Expose) break; }

/* clear the window */
/* XClearWindow(display,window);  */
}


/* gwxclear
   Clear current plot window
   Parameters:  None
*/
void
GWXCLEAR()
{  
   if(buffer == window) 
    {  XClearWindow(display,window); 
/*     printf("clearing window\n");   */
    }
    else
    {  XSetForeground(display, gc, bgcolor);
       XFillRectangle(display, buffer, gc, 0, 0, width, height);
       XSetForeground(display, gc, fgcolor); 
/*     printf("clearing buffer\n");  */
    };
}



/* gwxstatus
   Return current window status (position, size)
   Parameters:  
     xstart,ystart          (int*)   upper left corner coordinates in root
     xsize,ysize            (int*)   desired window size
*/
void
GWXSTATUS(xstart, ystart, xsize, ysize)
    int *xstart, *ystart, *xsize, *ysize;
{
    XWindowAttributes xwa;
    Window w_root, w_parent, *w_children;
    unsigned int nchildren; 

    XGetWindowAttributes(display, window, &xwa);
    *xsize = xwa.width;
    *ysize = xwa.height;
    *xstart = xwa.x;
    *ystart = xwa.y;
/*    printf("gwxstatus size x %d  y %d \n",*xsize,*ysize); */

/*  Position of window is meaningless under window manager, which remaps each
    window under a new parent. Find parent, if not root, get the parent's 
    position    */
    if(XQueryTree(display,window,&w_root,&w_parent,&w_children,&nchildren))
    {
      XFree((void*) w_children);  /* Free the list of child windows */
      if(w_parent!=DefaultRootWindow(display)) 

      {  XGetWindowAttributes(display, w_parent, &xwa);
         *xstart = xwa.x;
         *ystart = xwa.y;
      } 

    }
}


/* gwxclose
   Close current windows, buffers and display
   Parameters:  None
*/
void
GWXCLOSE()
{  

   XFreeCursor(display, cursor); 
   XFreeGC(display, gc); 
   if(pixmap != (Pixmap) NULL) XFreePixmap(display, pixmap);
   XCloseDisplay(display);
}


/* gwxdestroy
   Close current window and display
   Parameters:  None
*/
void
GWXDESTROY()
{  XDestroyWindow(display,window);
}


/* gwxflush
   Flush all pending graphics requests to the screen
   Parameters:  None
*/
void
GWXFLUSH()
{  if(buffer == pixmap)
    { XCopyArea(display, pixmap, window, gc, 0, 0, width, height, 0, 0);
/*    printf("copying pixmap to window x %d y %d\n",width,height); */  
    };

      XFlush(display);
}


/* gwxdisplaybuffer
   Switches background buffer onto foreground window, displaying accumulated
   graphics
   Parameters:  None
*/
void
GWXDISPLAYBUFFER()
{  GWXFLUSH();
}


/* gwxdrawtobuffer
   Switches graphics to draw to the background buffer
   Parameters:  None
*/
void
GWXDRAWTOBUFFER()
{
/* create and clear the pixmap */
   if(pixmap == (Pixmap) NULL) 
    { pixmap = XCreatePixmap(display, window, width, height, root_depth);
      XSetForeground(display, gc, bgcolor);
      XFillRectangle(display, pixmap, gc, 0, 0, width, height);
      XSetForeground(display, gc, fgcolor); 
    }

/* point graphics to the pixmap */
   buffer = pixmap;
/* printf("called gwxdrawtobuffer\n"); */
}

/* gwxdrawtowindow
   Switches graphics to draw to the foreground window
   Parameters:  None
*/
void
GWXDRAWTOWINDOW()
{  buffer = window;
/* printf("called gwxdrawtowindow\n"); */ 
}


/* gwxline
   Draw line from x1,y1 to x2,y2
   Parameters:  
     x1,y1            (int*)   starting position for line
     x2,y2            (int*)   ending position for line
*/
void
GWXLINE(x1, y1, x2, y2)
   int     *x1, *y1, *x2, *y2;
{  XDrawLine(display, buffer, gc, *x1, *y1, *x2, *y2);
}


/* gwxresize
   Resize screen window to x width, y height
   Parameters:  
     x           (int*)   new width
     y           (int*)   new height
*/
void
GWXRESIZE(x, y)
   int     *x, *y;
{  unsigned int xsiz,  ysiz;
   unsigned int xsize, ysize, xstart, ystart;
   int i;
   XEvent       event;
   long      event_mask;

/*   printf("called gwxresize\n");  */
   event_mask = ExposureMask | ResizeRedirectMask;


/* Clear the window events before resizing to setup for post resize Expose */
   while(XPending(display))
      XNextEvent(display, &event);

   xsiz = *x;
   ysiz = *y;
/*   printf("gwxresize to x %d  y %d\n",xsiz,ysiz); */
   XResizeWindow(display, window, xsiz, ysiz);
   XMapRaised(display, window);
   XFlush(display);

/* check for window size change */
   i = 0;
   while(1)
   {  GWXSTATUS(&xstart, &ystart, &xsize, &ysize);
      if( (xsize==xsiz) && (ysize==ysiz) )
        break;
/*        printf("resize status check count = %d\n",i++);  */
   }

/* update window size */
   width  = xsiz;
   height = ysiz;
/* destroy old back buffer pixmap and create new one in proper size */
   if(pixmap != (Pixmap) NULL) 
    { XFreePixmap(display, pixmap);
/*    printf("create new pixmap sized to x %d  y %d\n",width,height); */
      pixmap = XCreatePixmap(display, window, width, height, root_depth);
      if(buffer != window) buffer = pixmap;
/* clear the pixmap */
      XSetForeground(display, gc, bgcolor);
      XFillRectangle(display, pixmap, gc, 0, 0, width, height);
      XSetForeground(display, gc, fgcolor); 
    }


/* Raise the window and wait for an expose event to do anything in it */
/*   while(1)  
   { XWindowEvent(display, window, event_mask, &event);
   switch(event.type) {
   case Expose: 
       printf("Expose event \n");
       break; 
   case NoExpose: 
       printf("NoExpose event \n");
       break; 
   case ResizeRequest:
       printf("resizerequest event \n");
       break;
   }
   } */ 
}


/* gwxlinez
   Draw polyline on display window in current color and pen
   Parameters:  
     ix           (int*)   array of x coordinates on polyline
     iy           (int*)   array of y coordinates on polyline
     n            (int*)   number of coordinate points
    (see define for MAXPTS at start of this file)
*/
void
GWXLINEZ(ix, iy, n)
   int *n, *ix, *iy;
{  XPoint points[MAXPTS];
   int i;

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
   XDrawLines(display, buffer, gc, points, *n, CoordModeOrigin);
}


/* gwxpoly
   Draw filled polyline on display window in current color and pen
   Parameters:  
     x_coord      (int*)   array of x coordinates on polyline
     y_coord      (int*)   array of y coordinates on polyline
     n_coord      (int*)   number of coordinate points
    (see define for MAXPTS at start of this file)
*/
void
GWXPOLY(x_coord, y_coord, n_coord)
   int     *x_coord, *y_coord, *n_coord;
{  XPoint  points[MAXPTS];
   int     i,n;

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

   XFillPolygon(display, buffer, gc, points, n,
                Nonconvex, CoordModeOrigin);
}


/* gwxsetcolor
   Set foreground color to pixel value
   Parameters:  
     pixel     (int*)   pixel value (mapped in colormap)
*/
void
GWXSETCOLOR(pixel)
   int *pixel;
{  fgcolor = *pixel;
   XSetForeground(display, gc, fgcolor);
}



/* gwxsetbgcolor
   Set background color to pixel value
   Parameters:  
     pixel     (int*)   pixel value (mapped in colormap)
*/
void
GWXSETBGCOLOR(pixel)
   int *pixel;
{  bgcolor = *pixel;
   XSetForeground(display, gc, bgcolor);
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
void 
GWXCOLORNAME2RGB(int *red, int *grn, int *blu, 
                 int *nc, char *colorname, int len)
{
  XColor color_def;
  char cname[32];
  int i,n;

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
  if (!XParseColor(display, cmap,cname,&color_def)) {
    fprintf(stderr,"Xplot11.gwxcolorname2rgb: color name '%s' not found\n",
	    cname);
    printf("Xplot11.gwxcolorname2rgb: color name '%s' not found\n",cname);
    return;
  }
  *red = color_def.red  /256;
  *grn = color_def.green/256;
  *blu = color_def.blue /256;
}


/* gwxallocrgbcolor
   Allocate a color in colormap specified by r,g,b components 

   Parameters:  
     red,grn,blu   (int*)  input color components (0-255)
     ic            (int*)  returned color index or pixel value
*/
void 
GWXALLOCRGBCOLOR(int *red, int *grn, int *blu, int *ic)
{
  XColor color_def;

  color_def.red   = *red * 256;
  color_def.green = *grn * 256;
  color_def.blue  = *blu * 256;
  *ic = -1;

  if (!XAllocColor (display, cmap, &color_def)) {
    fprintf (stderr,"Xplot11.gwxallocrgbcolor: can't allocate color.\n");
    printf ("Xplot11.gwxallocrgbcolor: can't allocate color.\n");
    return;
  }
  *ic = color_def.pixel;
}


/* gwxfreecolor
   Free a color from color map
   Parameters:  
     pix (int*)  pixel in colormap
*/
void
GWXFREECOLOR(int *pix)
{     unsigned long lpix;

      lpix = *pix;
      XFreeColors(display,cmap,&lpix,1,0L);
}


/* gwxstring
   Draw string on display at specified position in current font and color
   Parameters:  
     x           (int*)   array of x coordinates on polyline
     y           (int*)   array of y coordinates on polyline
     string      (char*)  character string
     length      (int*)   length of string 

*/
void
GWXSTRING(x, y, string, length)
   int     *x, *y, *length;
   char    *string;
{  XDrawString(display, buffer, gc, *x, *y, string, *length);
}

 

/* gwxdash
   Set line drawing pattern to mask pattern
   Parameters:  
     mask     (int*)  integer mask value (bits set pen pattern for lines)
*/
void
GWXDASH(int* lmask)
{
char dashes[16];
int idash[16], nchrs, ioff, i;

    if(*lmask==0) 

      XSetLineAttributes(display, gc, line_width, 

			 LineSolid, CapButt, JoinMiter);
     else if(*lmask==-1)
      XSetLineAttributes(display, gc, line_width, 

			 LineSolid, CapButt, JoinMiter);
     else 

      {
      (void) MSKBITS(lmask,idash,&nchrs);
      for(i=1;i<=nchrs;i++) dashes[i-1]=idash[i-1];
/*    for(i=1;i<=nchrs;i++) printf("%d dashes %d\n",i,dashes[i-1]);   */

      ioff = 0;
      XSetDashes(display, gc, ioff, dashes, nchrs);
/* use LineOnOffDash  to only write "on" pixels in dashed lines (fg only) */
/* use LineDoubleDash to overwrite all pixels in dashed lines (fg AND bg) */
      XSetLineAttributes(display, gc, line_width, 

                         LineOnOffDash, CapButt, JoinMiter);
      }
}

 

/* gwxreset
   Reset graphics context to default
   Parameters:   None
*/
void
GWXRESET()
{
    XGCValues     gcv;
    unsigned long gcv_mask;

    line_width = 0;
    gcv.function   = GXcopy;
    gcv.foreground = fgcolor;
    gcv.background = bgcolor;
    gcv.line_width = line_width;
    gcv.line_style = LineSolid;
    gcv.cap_style  = CapButt;
    gcv.fill_style = FillSolid;
    gcv.join_style = JoinMiter; 
    gcv_mask = GCFunction  | GCForeground | GCBackground | GCLineWidth | 
               GCLineStyle | GCCapStyle   | GCFillStyle  | GCJoinStyle; 
    XChangeGC(display, gc, gcv_mask, &gcv);

}

 

/* gwxpen
   Set pen width for line drawing 
   Parameters:  
     ipen     (int*)  integer pen width
*/
void
GWXPEN(int* ipen)
{
    XGCValues     gcv;
    unsigned long gcv_mask;

    line_width = *ipen;
    gcv.line_width = line_width;
    gcv.cap_style = CapButt;
    gcv.join_style = JoinMiter; 
    gcv_mask = GCLineWidth | GCCapStyle | GCJoinStyle; 
    XChangeGC(display, gc, gcv_mask, &gcv);
}


/* gwxcurs
   Get cursor position and key pressed (waits for keypress or buttonpress) 
   Parameters:  
     x           (int*)   output cursor x coordinate
     y           (int*)   output cursor y coordinate
     state       (int*)   key pressed 

*/
void
GWXCURS(x, y, state)
   int *x, *y, *state;
{  XEvent    report;
   KeySym    key;
   int       count,buffer_len,last_event;
   long      event_mask;
   char      buffer[2];
/* int       idev = 1;  */

/* XRaiseWindow(display,window); */
   /* XSetInputFocus(display,window,RevertToNone,CurrentTime); */
/* warp to passed-in x,y (typ. previous cursor location) */
/*   printf("GWXCURS input x %d  y %d\n",*x,*y); */
   XWarpPointer(display, None, window, 0, 0, 0, 0, *x, *y); 

   event_mask = ExposureMask | ConfigureNotify | 
                KeyPressMask | ButtonPressMask;

/*  clear any pending events */   

   while(XPending(display))
      XNextEvent(display, &report);

   *state = 0;
   last_event = 0;
   while(! *state) 

 { XWindowEvent(display, window, event_mask, &report);
   switch(report.type) {
   case Expose:
      if(last_event != Expose) 

      { /* replot_(&idev); */
         XSetInputFocus(display,window,RevertToNone,CurrentTime);  }
      break;
   case ConfigureNotify:
      break;
   case ButtonPress:
      *state = report.xbutton.button;
      *x = report.xbutton.x;
      *y = report.xbutton.y;
      break;
   case KeyPress:
      *x = report.xkey.x;
      *y = report.xkey.y;
      buffer_len = 1;
      count = XLookupString(&report.xkey, buffer, buffer_len, &key, NULL);
      if (count == 0) printf("gwxcurs:  Zero length string returned.\n");
      *state = key;
      break;
   }
   last_event = report.type;
 }
/* XRaiseWindow(display,parent_window); */
/*   XSetInputFocus(display,parent_window,RevertToNone,CurrentTime); */
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
 









