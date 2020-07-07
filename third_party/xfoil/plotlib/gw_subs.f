C***********************************************************************
C    Module:  gw_subs.f
C 
C    Copyright (C) 1996 Harold Youngren, Mark Drela 
C 
C    This library is free software; you can redistribute it and/or
C    modify it under the terms of the GNU Library General Public
C    License as published by the Free Software Foundation; either
C    version 2 of the License, or (at your option) any later version.
C
C    This library is distributed in the hope that it will be useful,
C    but WITHOUT ANY WARRANTY; without even the implied warranty of
C    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
C    Library General Public License for more details.
C
C    You should have received a copy of the GNU Library General Public
C    License along with this library; if not, write to the Free
C    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
C 
C    Report problems to:    guppy@maine.com 
C                        or drela@mit.edu  
C***********************************************************************
C
C***********************************************************************
C --- Xplot11 driver for X windows plotting to screen
C
C     Version 4.46 11/28/01
C
C     Notes:  These routines handle the interface to the graphics screen
C             window.  Not normally called by the user.
C             The interface file Xwin.c is needed to link to X11 
C***********************************************************************

      subroutine gw_setup(scrn_fraction)
C---Sets default size of screen window
C    scrn_fraction   relative size (0->1) of graphics window to root window 
C     size defaults to 520x670 if scrn_fraction<0.0 and aspect ratio is set 
C     by 8.5"x11.0" page size.
C
      include 'pltlib.inc'
      DATA w1size, w2size / DEFAULT_PAGEHEIGHT, DEFAULT_PAGEWIDTH /
C
      G_SCRNFRAC = abs(scrn_fraction)
      G_SCRNFRAC = MIN(1.0,G_SCRNFRAC)
      IF(scrn_fraction .EQ. 0.0) G_SCRNFRAC = DEFAULT_SCREENFRACTION
C
C---set default window size
      if(scrn_fraction .GE. 0.0) then
        X_PAGE = w1size
        Y_PAGE = w2size
        I_PAGETYPE = Page_Landscape
      else
        X_PAGE = w2size
        Y_PAGE = w1size
        I_PAGETYPE = Page_Portrait
      endif
C
C---window size in absolute units (pseudo-inches)
      X_WIND = X_PAGE
      Y_WIND = Y_PAGE
C
C---location of upper left window corner
      GX_LOC = 0
      GY_LOC = 0
C
C---location of plot origin on window
      GX_ORG = 0
      GY_ORG = 0
C
      return
      end



      subroutine gw_init
C---Initializes graphics screen plotting and global plot variables
      include 'pltlib.inc'
C
      ixsiz = GX_SIZ
      iysiz = GY_SIZ
      ixstart = GX_LOC
      iystart = GY_LOC
      scrnfrac = MIN(1.0,G_SCRNFRAC)
      A_RATIO = Y_WIND/X_WIND
C
      if(.NOT.LGW_OPEN) then
C---Open display and window if not already open
        call gwxopen(iwidth,iheight,idepth)
        G_WIDTH  = iwidth
        G_HEIGHT = iheight
        G_DEPTH  = idepth
C---Adjust window size if fraction of root window is specified
        if(scrnfrac.GT.0.0) then
          ix = ifix(scrnfrac*float(G_WIDTH))
          iy = ifix(scrnfrac*float(G_HEIGHT))
          if(ix*A_RATIO.LT.iy) then
            ixsiz = ix
            iysiz = ifix(A_RATIO*float(ix))
           else
            iysiz = iy
            ixsiz = ifix(float(iy)/A_RATIO)
          endif
        endif
        ixsiz = MIN(ixsiz,G_WIDTH)
        iysiz = MIN(iysiz,G_HEIGHT)
        call gwxwinopen(ixstart,iystart,ixsiz,iysiz)
C---Update flags for reference
        LGW_OPEN = .TRUE.
C---Graphics window color is not used if less than 4 bits (16 colors) 
C   are available
        LGW_COLOR = (G_DEPTH.gt.4)
        if(.NOT.LGW_COLOR) then
          write(*,*) 'Warning: color depth < 4 bits, color not used...'
        endif
C
C---initial cursor warp will be to middle of window
        GX_SIZ = ixsiz
        GY_SIZ = iysiz
        GX_CRS = GX_ORG + GX_SIZ/2
        GY_CRS = GY_ORG + GY_SIZ/2
C
       else 
C
C---Window already open, resize if specified
        if(LGW_RESIZE) then
C
          ixorig = ixsiz
          iyorig = iysiz
C---Resize window if new fractional size of root window is specified
          if(scrnfrac.GT.0.0) then
            ix = ifix(scrnfrac*float(G_WIDTH))
            iy = ifix(scrnfrac*float(G_HEIGHT))
            if(ix*A_RATIO.LT.iy) then
              ixsiz = ix
              iysiz = ifix(A_RATIO*float(ix))
             else
              iysiz = iy
              ixsiz = ifix(float(iy)/A_RATIO)
            endif
           else
            write(*,*) 'Warning: scrnfrac=0 specified, ignoring...' 
           stop
          endif
          ixsiz = MIN(ixsiz,G_WIDTH)
          iysiz = MIN(iysiz,G_HEIGHT)
          call gwxresize(ixsiz,iysiz)
          call gwxstatus(ixstart,iystart,ixsz,iysz)
          if(ixsz.NE.ixsiz .OR. iysz.NE.iysiz) then
           write(*,*) 'Window resize fails, using old window size'
           ixsiz = ixorig
           iysiz = iyorig
          endif
C---Update flags for reference
          LGW_RESIZE  = .FALSE.
          LGW_CHANGED = .TRUE.
C
        else
C
C...Graphics window can be moved & resized, so reset position & size parameters
         call gwxstatus(ixstart,iystart,ixsiz,iysiz)
         LGW_CHANGED = (ixsiz.ne.GX_SIZ .or. iysiz.ne.GY_SIZ)
C
        endif
C
        call gwxclear
      endif
C
C---Update size and origin
      GX_LOC = ixstart
      GY_LOC = iystart
      GX_SIZ = ixsiz
      GY_SIZ = iysiz
C
C---initial cursor warp will be to middle of window
cc      GX_CRS = GX_ORG + GX_SIZ/2
cc      GY_CRS = GY_ORG + GY_SIZ/2
C
C...Set new scale so that each window dimension is as big as possible 
C   but no bigger than X_PAGE or Y_PAGE.
      G_SCALE = max( float(GX_SIZ-1)/X_PAGE,
     &               float(GY_SIZ-1)/Y_PAGE  )
C
C...New window dimensions
      X_WIND = float(GX_SIZ-1)/G_SCALE
      Y_WIND = float(GY_SIZ-1)/G_SCALE
C
      return
      end


      subroutine gw_line(X1,Y1,X2,Y2)
C---Plots line segment to screen, 
C   specified in absolute coordinates from X1,Y1 to X2,Y2
      include 'pltlib.inc'
C
      if(LGW_OPEN) then
C...reverse Y axis for X11 plotting
        ix1 =           GX_ORG + ifix(G_SCALE*X1)
        ix2 =           GX_ORG + ifix(G_SCALE*X2)
        iy1 = GY_SIZ - (GY_ORG + ifix(G_SCALE*Y1)) - 1
        iy2 = GY_SIZ - (GY_ORG + ifix(G_SCALE*Y2)) - 1
        call gwxline(ix1,iy1,ix2,iy2)
      endif
C
      return
      end



      subroutine gw_curs(X,Y,kchar)
C---Gets location of mouse in graphics window in 
C---absolute coordinates.  Returns position when mouse button is pressed 
C---or keyboard key is pressed.
C
      include 'pltlib.inc'
C
      if(LGW_OPEN) then
C
C------ set previous cursor position (if any) for warping destination
        ix =           GX_ORG + GX_CRS
        iy = GY_SIZ - (GY_ORG + GY_CRS)
C
        call gwxcurs(ix,iy,kchar)
        GX_CRS =          ix - GX_ORG
        GY_CRS = GY_SIZ - iy - GY_ORG
C
        X = float(GX_CRS)/G_SCALE
        Y = float(GY_CRS)/G_SCALE
C
      endif
      return
      end


      subroutine gw_cursc(X,Y,ibtn)
C---Gets current location of mouse in graphics window in 
C---absolute coordinates.  Returns current position while mouse button 
C---is pressed, with ibtn=1. When mouse button is released ibtn=0.  
C---
C---
      include 'pltlib.inc'
      integer ibtn
C
      if(LGW_OPEN) then
C
C------ set previous cursor position (if any) for warping destination
        ix =           GX_ORG + GX_CRS
        iy = GY_SIZ - (GY_ORG + GY_CRS)
C
        call gwxcursc(ix,iy,ibtn)
        GX_CRS =          ix - GX_ORG
        GY_CRS = GY_SIZ - iy - GY_ORG
C
        X = float(GX_CRS)/G_SCALE
        Y = float(GY_CRS)/G_SCALE
C
      endif
      return
      end


      subroutine gw_revflag
C---Gets reverse video flag from users environment settings
      include 'pltlib.inc'
C
      call gwxrevflag(irev)
      LGW_REVVIDEO = (irev.ne.0) 
C
      return
      end


      subroutine gw_endplot
C---Flushes current graphics to window and returns control to 
C   graphics window temporarily
      include 'pltlib.inc'
C
      if(LGW_OPEN) then
        call gwxflush
      endif
C
      return
      end


      subroutine gw_pen(ipen)
C...Sets pen width for screen display
      include 'pltlib.inc'
      dimension ipmap(5)
c     data ipmap / 0, 0, 2, 3, 4 /  <- too wide!
      data ipmap / 0, 0, 1, 1, 2 /
C
      ip = max(1,ipen)
      ip = min(5,ip)
      if(LGW_OPEN) then
        call gwxpen(ipmap(ip))
      endif
C
      return
      end
 

      subroutine gw_linepattern(ipat)
C...Sets pen width for screen display
      include 'pltlib.inc'
C
      if(LGW_OPEN) call gwxdash(ipat)
      return
      end
 

      subroutine gw_flush
C...Flushes out buffered plotting calls to display 
      include 'pltlib.inc'
C
      if(LGW_OPEN) call gwxflush
      return
      end
  

      subroutine gw_drawtoscreen
C...Sets graphics destination to screen
      include 'pltlib.inc'
C
      if(LGW_OPEN) call gwxdrawtowindow
      return
      end
  

      subroutine gw_drawtobuffer
C...Sets graphics destination to background buffer
      include 'pltlib.inc'
C
      if(LGW_OPEN) call gwxdrawtobuffer
      return
      end
  

      subroutine gw_showbuffer
C...Displays graphics drawn into background buffer on screen
C   This does not change the current destination for drawing commands
      include 'pltlib.inc'
C
      if(LGW_OPEN) call gwxdisplaybuffer
      return
      end
  

      subroutine gw_clear
C...Erases graphics display
      include 'pltlib.inc'
C
      if(LGW_OPEN) call gwxclear
      return
      end
 

      subroutine gw_setsize(nx,ny)
C...Resizes X-Window X,Y size to nx,ny in pixels
      include 'pltlib.inc'
C
      if(nx.LE.10) then
        write(*,*) 'No Resize: Window size too small: ',nx,ny
        return
      endif
C
C...Get current window parameters if display is open
      if(LGW_OPEN) then
        call gwxstatus(ixstart,iystart,ixsiz,iysiz)
        LGW_CHANGED = (ixsiz.ne.GX_SIZ .or. iysiz.ne.GY_SIZ)
        GX_LOC = ixstart
        GY_LOC = iystart
        GX_SIZ = ixsiz
        GY_SIZ = iysiz
      endif
C
C...Resize and replot the window contents 
      if(LGW_OPEN) then
        call gwxresize(nx,ny)
        call gwxstatus(ixstart,iystart,ixsz,iysz)
          if(ixsz.NE.ixsiz .OR. iysz.NE.iysiz) then
           write(*,*) 'Window resize fails, using old window size'
           ixsz = ixsiz
           iysz = iysiz
          endif
C---Update flags and size for reference
        LGW_RESIZE  = .FALSE.
        LGW_CHANGED = .TRUE.
        GX_SIZ = ixsz
        GY_SIZ = iysz
C
C---initial cursor warp will be to middle of window
        GX_CRS = GX_ORG + GX_SIZ/2
        GY_CRS = GY_ORG + GY_SIZ/2
C---Set new scale so that each window dimension is as big as possible 
C   but no bigger than X_PAGE or Y_PAGE.
        G_SCALE = max( float(GX_SIZ-1)/X_PAGE,
     &                 float(GY_SIZ-1)/Y_PAGE  )
C---New window dimensions
        X_WIND = float(GX_SIZ-1)/G_SCALE
        Y_WIND = float(GY_SIZ-1)/G_SCALE
C---refresh the newly sized window
        call REPLOT(1)
      endif
      return
      end
 

      subroutine gw_getsize(nx,ny,ppi)
C...Returns X Window X,Y size in pixels, 
C   and plotting scale in pixels/(absolute coordinate)
      include 'pltlib.inc'
C
      if(LGW_OPEN) then
        nx = GX_SIZ
        ny = GY_SIZ
        ppi = G_SCALE
      endif
      return
      end
 

      subroutine gw_close
C---Closes graphics window
      include 'pltlib.inc'
C
      if(LGW_OPEN) then
        call gwxclose
        LGW_OPEN = .FALSE.
      endif
      return
      end



      subroutine gw_cname2rgb(colorname,ired,igrn,iblu)
C...Determines RGB color components of color defined the color name
C   using the X window color database.
C
C    colorname = name of color (must be in X windows rgb.txt file)
C
C   The returned color components are 
C    ired = 0-255 red   component (-1 for no color)
C    igrn = 0-255 green component
C    iblu = 0-255 blue  component
C
      include 'pltlib.inc'
      character colorname*(*), cname*80
      ired = -1
      igrn =  0
      iblu =  0
ccc   nc = len(colorname)
      cname = colorname
      call g_strip(cname,nc)
      if(LGW_OPEN) then 
        call gwxcolorname2rgb(ired,igrn,iblu,nc,cname)
      endif
      return
      end


      subroutine gw_allocrgbcolor(ired,igrn,iblu,icolorindex)
C...Allocates plotting color as defined by RGB components 
C   The component colors are 
C    ired = 0-255 red   component
C    igrn = 0-255 green component
C    iblu = 0-255 blue  component
C
C    icolorindex = returned color index of allocated color in window colormap
C
      include 'pltlib.inc'
      ic = -1
      if(LGW_OPEN .AND. LGW_COLOR) then 
        ir = ired
        ig = igrn
        ib = iblu
        call gwxallocrgbcolor(ir,ig,ib,ic)
        icolorindex = ic
      endif
      return
      end


      subroutine gw_color(icolor)
C...Sets foreground plotting color 
C   The colors are defined by the colormap
C
C   The color in the colormap table and colorindex go from 0 -> N_COLOR
C
      include 'pltlib.inc'
c
      if(LGW_OPEN .AND. LGW_COLOR) then 
c
        icol = mod(icolor,N_COLOR+1) 
        if(icolor.le.0 .OR. icolor.gt.N_COLOR) then
          write(*,*) '*** gw_color - color index out of bounds ',icolor
          return
        endif
C
C...Reverse white and black only in video window if LGW_REVVIDEO is set
        if(LGW_REVVIDEO) then
          if(icolor.eq.2) icol = 1
          if(icolor.eq.1) icol = 2
        endif
C
C...Check colorindex<0 for this color, if so this is an unallocated color
        ic = G_COLOR_CINDEX(icol)
C
C...If color is unallocated, allocate it using the RGB components
        if (ic.lt.0) then
          irgb = COLOR_RGB(icol)
          irg = irgb/256 
          ired = irg/256 
          igrn = irg  - 256*ired
          iblu = irgb - 256*irg
          call gw_allocrgbcolor(ired,igrn,iblu,ic)
C
C...This gives the colorindex in the screen colormap (or -1 for no allocation)
          if(ic.ge.0) G_COLOR_CINDEX(icol) = ic
        endif
C
C...Now, if colorindex is valid, set the color
        if(ic.ge.0) call gwxsetcolor(ic)
c
      endif
      return
      end


      subroutine gw_bgcolor(icolor)
C...Sets background plotting color 
C   The colors are defined by the colormap
C
C   The color in the colormap table and colorindex go from 0 -> N_COLOR
C
      include 'pltlib.inc'
c
      if(LGW_OPEN .AND. LGW_COLOR) then 
c
        icol = mod(icolor,N_COLOR+1) 
        if(icolor.le.0 .OR. icolor.gt.N_COLOR) then
          write(*,*) '*** gw_bgcolor - color index out of bounds ',
     &               icolor
          return
        endif
C
C...Reverse white and black only in video window if LGW_REVVIDEO is set
        if(LGW_REVVIDEO) then
          if(icolor.eq.2) icol = 1
          if(icolor.eq.1) icol = 2
        endif
C
C...Check colorindex<0 for this color, if so this is an unallocated color
        ic = G_COLOR_CINDEX(icol)
C
C...If color is unallocated, allocate it using the RGB components
        if (ic.lt.0) then
          irgb = COLOR_RGB(icol)
          irg = irgb/256 
          ired = irg/256 
          igrn = irg  - 256*ired
          iblu = irgb - 256*irg
          call gw_allocrgbcolor(ired,igrn,iblu,ic)
C
C...This gives the colorindex in the screen colormap (or -1 for no allocation)
          if(ic.ge.0) G_COLOR_CINDEX(icol) = ic
        endif
C
C...Now, if colorindex is valid, set the color
        if(ic.ge.0) call gwxsetbgcolor(ic)
c
      endif
      return
      end


      subroutine gw_newcmap
C...Sets up new colormap, old colormap is flushed
      include 'pltlib.inc'
      if(LGW_OPEN .AND. LGW_COLOR) then 
        do i = 1, N_COLOR
          ic = G_COLOR_CINDEX(i)
          if(ic.gt.0) call gwxfreecolor(ic)
        end do
      endif
      return
      end



      subroutine gw_polyline(X,Y,n,ifill)
C...Plots polyline (optionally filled) to screen in foreground color
C     X,Y    polyline  X,Y absolute coordinates
C     n      number of X,Y points
C     ifill  fill flag, 0 for no fill, 1 for filled polyline
C
      include 'pltlib.inc'
      dimension X(n), Y(n), ix(MaxPolyLine), iy(MaxPolyLine)
C
      if(n.GT.MaxPolyLine) then
        write(*,*) 'gw_polyline: array overflow.  Increase MaxPolyline.'
        return
      endif
c
      if(n.LE.1) return
      if(LGW_OPEN) then
C...reverse Y axis for X11 plotting
        do i = 1, n
          ix(i) =           GX_ORG + ifix(G_SCALE*X(i))
          iy(i) = GY_SIZ - (GY_ORG + ifix(G_SCALE*Y(i))) - 1
        end do
        if(ifill.EQ.0) then
          call gwxlinez(ix,iy,n)
         else
          call gwxpoly(ix,iy,n)
        endif
      endif
      return
      end


 
      subroutine g_strip(strng,n)
      character strng*(*)
c
      nc = len(strng)
      n = nc
      if (n.le.1) return
c
c---- Strips leading blanks off string
      if(strng(1:1).EQ.' ') then
c
c---- find first non-blank character
        do i=2, nc
          if(strng(i:i).NE.' ') go to 10
        end do
        go to 20
 10     ioff = i
C
C---- shift STRNG so first character is non-blank
        strng(1:nc-ioff) = strng(ioff:nc)
C---- pad tail of STRNG with blanks
        strng(nc-ioff+1:nc) = ' '
c
      endif
c
c---- find the index of the last non-blank charater in NAME
 20   do n=nc, 1, -1
        if(strng(n:n).NE.' ') go to 40
      end do
 40   return
      end

