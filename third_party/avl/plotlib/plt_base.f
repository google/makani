C***********************************************************************
C    Module:  plt_base.f
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
***********************************************************************
C Xplot11 *
C**********
C
C     Dedicated to perpetuating ugly PLOT-10 and Versatec software 
C     into the 21st century...
C
C     This library supports interactive graphics and hardcopy output using
C     the interfaces defined in gw_subs.f and ps_subs.f.  
C     Currently:   gw_subs supports X window graphics in B&W or color
C                  ps_subs supports B&W or color hardcopy to Postscript file
C     
C     Version 4.46 11/28/01
C
C     Notes:   
C
C***********************************************************************
 

      subroutine PLINITIALIZE
C
C---Plot package initialization routine
C   Must be called before any color plot calls
C
      include 'pltlib.inc'
C
      GX_SIZ = 1
      GY_SIZ = 1
C---Check for user default for background and set up a colormap
      call gw_revflag
      call colormapdefault
C
      return
      end


      subroutine PLOPEN(relsize,nunit,idev)
C
C---Plot initialization routine
C
C   Must be called before EACH page plot
C
C    relsize   Fractional size of X-graphics window relative to screen
C              if relsize < 0 the graphics page is in Portrait  mode 8.5x11
C              if relsize >=0 the graphics page is in Landscape mode 11x8.5
C
C    nunit     Postscript output file specifier
C         < 0     Postscript output enabled to individual files plotNNN.ps 
C                 on unit 80 where NNN is cumulative plot number
C         = 0     Postscript output enabled to file plot.ps  on unit 80
C         = NNN   Postscript output enabled to file plotUUU.ps on unit UUU
C
C    idev      Plotting output selector
C
C                idev    X-window   PostScript
C                ----    --------   ----------
C                  1        x
C                  2                  B & W
C                  3        x         B & W
C                  4                  Color
C                  5        x         Color
C
C        Note: idev<1 or idev>5  gives only X-window output
C              (for pseudo-compatibility with old PLTLIB versions)
C              ((if you squint real hard))
C
      include 'pltlib.inc'
C
      SAVE ifirst, relsize_save, nunit_save
      DATA ifirst / 0 /
C
      I_DEV = idev
      if(idev.GT.5)  I_DEV = 1
C
C---- control flags  (gw_init will set LGW_COLOR = T if screen has color)
      LGW_GEN   = (I_DEV.EQ.1 .OR. I_DEV.EQ.3 .OR. I_DEV.EQ.5)
      LPS_GEN   = (I_DEV.GE.2)
      LPS_COLOR = (I_DEV.GE.4)
C
C
C---- clear primitives counter
      N_PRIM = 0
C
      if(ifirst.EQ.0)  then
C------ First-ever PLOPEN call
C
        ifirst = 1
C...graphics window
        LGW_OPEN   = .FALSE.
        call gw_setup(relsize)
        LGW_RESIZE = .FALSE.
C
C...PostScript (file not yet opened, and no unstroked page exists)
        NPS_UNIT = -1
        N_PAGES  = 0
        call ps_setup(nunit)
C...No zooming to start off
        call CLRZOOM
C...Primitives file initializers
        NPRIM_UNIT = NPRIM_UNIT_DEFAULT
        LPRIM_OPEN = .FALSE.
C
      else
C------ Subsequent PLOPEN call
C...if a postscript file has been opened we need to end the page with 
C   bounding box information
        if(LPS_OPEN) call ps_endpage
C
        if(abs(relsize-relsize_save) .GT. 0.01) then
C...relsize specfied has changed from previous PLOPEN call.. 
C...re-setup aspect ratio and prepare to resize current graphics window
          call gw_setup(relsize)
          LGW_RESIZE = .TRUE.
        endif
C
        if(nunit.NE.nunit_save .OR. nunit.LT.0) then
C...nunit changed from previous PLOPEN call or individual plots are desired
C   close current PostScript file
          call ps_close
C...set up for new PostScript file
          call ps_setup(nunit)
        endif
C
C...If the prims file is open, rewind to be ready to write from the beginning
        if(LPRIM_OPEN) then
          rewind(NPRIM_UNIT)
        endif
C
      endif
C
C...Initialize selected plot devices
      if(LGW_GEN) call gw_init
      if(LPS_GEN) call ps_init
C
      if(LGW_GEN) then 
        if(LGW_CHANGED) write(*,1000) X_WIND, Y_WIND
 1000   format(/1x,'X-window size changed to ',f6.2,'" x',f6.2,'"')
      endif 
C
C...Initialize line plot width, pattern, color
      I_PEN = 1
      I_PAT = -1
      I_CLR = 1
      call set_pen(I_PEN)
      call set_pat(I_PAT)
      call set_color(I_CLR)
C
      X_SCALE = 1.0
      Y_SCALE = 1.0
      X_ORG = 0.
      Y_ORG = 0.
      X_LST = 0.
      Y_LST = 0.
C
C...No initial user clipping (clipping only to page size)
      CLP_XMIN = 0.
      CLP_XMAX = X_PAGE
      CLP_YMIN = 0.
      CLP_YMAX = Y_PAGE
C
C
C...Save current call parameters
      relsize_save = relsize
      nunit_save   = nunit
C
      return
      end


      subroutine REPLOT(idev)
C
C...Replots plot primitives saved in logging array since last PLOPEN call
c
C   idev - as defined in PLOPEN header 
C
      include 'pltlib.inc'
      dimension xpoly(MaxPolyLine), ypoly(MaxPolyLine)
C
      idev_old = I_DEV
C
C
      I_DEV = idev
      if(idev.LE.0 .OR. idev.GT.5)  I_DEV = idev_old
C
C...Control flags  (LGW_COLOR should be already set for current screen)
      LGW_GEN   = (I_DEV.EQ.1 .OR. I_DEV.EQ.3 .OR. I_DEV.EQ.5)
      LPS_GEN   = (I_DEV.GE.2)
      LPS_COLOR = (I_DEV.GE.4)
C
C...Re-Initialize selected plotting devices
      if(LGW_GEN) call gw_init
      if(LPS_GEN) call ps_init
C
C...Reset plot globals for this plot
      I_PEN =  1
      I_PAT = -1
      I_CLR =  1
      call set_pen(I_PEN)
      call set_pat(I_PAT)
      call set_color(I_CLR)
C
      X_SCALE = 1.0
      Y_SCALE = 1.0
      X_ORG = 0.
      Y_ORG = 0.
      X_LST = 0.
      Y_LST = 0.
C
C...No initial user clipping (clipping only to window limits)
      CLP_XMIN = 0.
      CLP_XMAX = X_PAGE
      CLP_YMIN = 0.
      CLP_YMAX = Y_PAGE
C
      IPOLY   = 0
      ICLPMIN = 0            
C...Now, go through all the stored plot primitives
      ICNT = -1
 1    call getprim(ICNT,ITYP,IVAL,XVAL,YVAL)
C
      if(ICNT.LT.0) go to 10
C
        if(ITYP.EQ.PageCommand) then 
           if(IVAL.EQ.-999) then
             if(LPS_GEN) call ps_endpage
           endif
C
         else if(ITYP.EQ.PlotCommand) then 
           call plot_1(XVAL,YVAL,IVAL)
C
C...Not currently using scale info in replots, all X,Y are absolute (HHY)
         else if(ITYP.EQ.ScaleCommand) then
           call set_scl(XVAL,YVAL)
C
         else if(ITYP.EQ.PenCommand) then 
           I_PEN = IVAL
C
         else if(ITYP.EQ.PatternCommand) then
           I_PAT = IVAL
C
         else if(ITYP.EQ.ColorCommand) then
           I_CLR = IVAL
C
         else if(ITYP.EQ.PolylinePointCommand) then
           IPOLY = IPOLY+1
           if(IPOLY.GT.MaxPolyline) then
            write(*,*) '*** Error - too many polyline points'
            stop
           endif
           xpoly(IPOLY) = XVAL
           ypoly(IPOLY) = YVAL
C
         else if(ITYP.EQ.PolylineDrawCommand) then
           IPOLY = IPOLY+1
           if(IPOLY.GT.MaxPolyline) then
            write(*,*) '*** Error - too many polyline points'
            stop
           endif
           xpoly(IPOLY) = XVAL
           ypoly(IPOLY) = YVAL
           ifill = IVAL
           call polyline_1(xpoly,ypoly,IPOLY,ifill)
           IPOLY = 0            
C
         else if(ITYP.EQ.MinClipCommand) then
           CLPMINX = XVAL
           CLPMINY = YVAL
           ICLPMIN = 1            
C
         else if(ITYP.EQ.MaxClipCommand) then
           if(ICLPMIN.NE.1) then
            write(*,*) '*** Error - no previous MinClip stored'
            stop
           endif
           call set_clip(CLPMINX,CLPMINY,XVAL,YVAL)
           ICLPMIN = 0            
C
         else
           write(*,*) '? REPLOT -- Illegal Command:', ITYP
C
        endif
        go to 1
C
 10   if(IPOLY.NE.0) then
        write(*,*) '? REPLOT -- No end to polyline command.'
        stop
      endif
      call PLFLUSH
C
      I_DEV = idev_old  
      LGW_GEN   = (mod(I_DEV,2) .EQ. 1)
      LPS_GEN   = (I_DEV .GE. 2)
      LPS_COLOR = (I_DEV .GE. 4)
C
      return
      end


      subroutine PLCLOSE
C---Closes all plotting, no more plots...
C    closes any open postscript files
C    closes X window
C    closes and deletes log file (if used) 
      include 'pltlib.inc'
C
      call ps_endpage
      call gw_close
      call ps_close
      if(LPRIM_OPEN) then
        close(unit=NPRIM_UNIT,status='DELETE')
      endif
      return
      end


      subroutine PLEND
C---Ends current plot, 
C    finishes off current postscript plot page
C    ends current X window plot, flushes to display 
      include 'pltlib.inc'
C
      call putprim(PageCommand,-999,0.,0.)
      if(LGW_GEN) call gw_endplot
      if(LPS_GEN) call ps_endpage
      return
      end




      subroutine PLOT(x,y,icode)
C---Basic plotting routine, does moves and draws in user coordinates
C   with optional reorigin, also can end this plot or all plotting
C    x,y    coordinates in user units
C    icode  function code (integer)
C           3   relative move to x,y
C           2   relative line to x,y	     
C          -2   relative line to x,y and re-origin plotting to x,y
C          -3   relative move to x,y and re-origin plotting to x,y
C        -999   end this plot page
C        +999   end all plotting, close graphics window
C
      include 'pltlib.inc'
C
C...Convert plot coordinates to absolute units and plot
      XABS = xusr2ABS(x)
      YABS = yusr2ABS(y)
      CALL PLOTABS(XABS,YABS,icode)
      return
      end



      subroutine PLOTABS(x,y,icode)
C---Absolute plotting routine, does moves and draws in absolute coordinates
C   with optional reorigin, also can end this plot or all plotting
C
C    X,Y    coordinates in absolute units
C    icode  function code (integer)
C           3   relative move to X,Y
C           2   relative line to X,Y	     
C          -2   relative line to X,Y and re-origin plotting to X,Y
C          -3   relative move to X,Y and re-origin plotting to X,Y
C        -999   end this plot page
C        +999   end all plotting, close graphics window
C
      include 'pltlib.inc'
      logical LCODE_OK
C
      icabs = abs(icode)
C
      LCODE_OK = ( icabs.EQ.2  .OR. icabs.EQ.3  .OR.
     &             icabs.EQ.999 ) 
C
      if(.NOT. LCODE_OK) then
        write(*,*) 'PLOTABS: Unknown function code: ',icode
        write(*,*) '      at point X,Y =',X,Y
        return
      endif 
C
C---Check for end of plot page
      if    (icode.EQ.-999) then
        call PLEND
C---Check for end of ALL plotting
      elseif(icode.EQ.+999) then
        call PLCLOSE
C
      else
C...Store plot primitive
        call putprim(PlotCommand,icode,X,Y)
C.....Do draw/move call with absolute coordinates
        call plot_1(X,Y,icode)
C
      endif
C
      return
      end




      subroutine POLYLINE(x,y,n,ifill)
C---Basic polyline plotting routine, input in user coordinates
C    x,y    coordinate arrays in user units
C    n      number of x,y points
C    ifill  fill flag, 0 for no fill, 1 for filled polygon
C
      include 'pltlib.inc'
      dimension x(n), y(n)
      dimension XABS(MaxPolyLine), YABS(MaxPolyLine)
C
      if(n.GT.MaxPolyline) then
        write(*,*) '*** Error - too many polyline points'
        stop
      endif
C
      if(n.LE.1) return
C...Convert coordinates to absolute coordinates
      do i=1, n
        XABS(i) = xusr2ABS(x(i))
        YABS(i) = yusr2ABS(y(i))
      end do
C...Plot polyline in absolute coordinates
      call POLYLINEABS(XABS,YABS,n,ifill)
      return
      end


      subroutine POLYLINEABS(X,Y,n,ifill)
C---Basic polyline plotting routine, input in absolute coordinates
C    X,Y    coordinate arrays in absolute units
C    n      number of X,Y points
C    ifill  fill flag, 0 for no fill, 1 for filled polygon
C
      include 'pltlib.inc'
      dimension X(n), Y(n)
C
      if(n.LE.1) return
C
C...Store polyline primitives in stored plot array and do polyline plot call
      icode = ifill
      do i=1, n-1
        call putprim(PolylinePointCommand,icode,X(i),Y(i))
      end do
      call putprim(PolylineDrawCommand,icode,X(n),Y(n))
C
C...plot polyline
      call polyline_1(X,Y,n,ifill)
      return
      end



      subroutine GETPEN(ipen)
C...Gets current pen width in pixels 
      include 'pltlib.inc'
      ipen = I_PEN
      return
      end

      subroutine NEWPEN(ipen)
C...Sets line width from 1 to 5 (pixels) 
      include 'pltlib.inc'
      if(ipen.EQ.I_PEN) return
c
      ip = ipen
      if (ip.GT.5) ip = 5
      if (ip.LT.0) ip = 1
      I_PEN = ip
C...Install pen command into display primitives list
      call putprim(PenCommand,ip,0.,0.)
      return
      end



      subroutine GETPAT(ipat)
C...Gets current line pattern as integer bit pattern
      include 'pltlib.inc'
      ipat = I_PAT
      return
      end


      subroutine NEWPAT(ipat)
C...Sets line pattern using bit pattern in lower 16 bits of ipat
      include 'pltlib.inc'
      if(ipat.EQ.I_PAT) return
c
      I_PAT = ipat
C...Install pattern command into display primitives list
      call putprim(PatternCommand,ipat,0.,0.)
      return
      end




      subroutine GETORIGIN(XORG,YORG)
C...Gets origin of user system in absolute (page) units
      include 'pltlib.inc'
C
      XORG = X_ORG
      YORG = Y_ORG
      return
      end


      subroutine NEWORIGIN(XORG,YORG)
C...Sets origin of user system in absolute (page) units
      include 'pltlib.inc'
C
      X_ORG = XORG
      Y_ORG = YORG
      return
      end


      subroutine GETFACTORS(xscale,yscale)
C...Gets current scale factors in user units
      include 'pltlib.inc'
      xscale = X_SCALE
      yscale = Y_SCALE
      return
      end


      subroutine NEWFACTOR(scale)
C...Sets both plot scale factors to scale
      include 'pltlib.inc'
      call set_scl(scale,scale)
C...Install scale command into display primitives list
      call putprim(ScaleCommand,0,scale,scale)
      return
      end


      subroutine NEWFACTORS(xscale,yscale)
C...Sets plot scale factors
      include 'pltlib.inc'
      call set_scl(xscale,yscale)
C...Install scale command into display primitives list
      call putprim(ScaleCommand,0,xscale,yscale)
      return
      end


      subroutine GETUSERTRANS(XORG,YORG,xscale,yscale)
C...Gets origin and scale factors for user->absolute coordinate transform
      include 'pltlib.inc'
C
      XORG = X_ORG
      YORG = Y_ORG
      xscale = X_SCALE
      yscale = Y_SCALE
      return
      end


      subroutine NEWUSERTRANS(XORG,YORG,xscale,yscale)
C...Sets origin and scale factors for user->absolute coordinate transform
      include 'pltlib.inc'
C
      X_ORG = XORG
      Y_ORG = YORG
      X_SCALE = xscale
      Y_SCALE = yscale
      return
      end


      subroutine GETLASTXY(x,y)
C...Return last x,y plotting location in user coordinates
      include 'pltlib.inc'
C
      call GETLASTXYABS(XABS,YABS)
      x = XABS2usr(XABS)
      y = YABS2usr(YABS)
      return
      end


      subroutine GETLASTXYABS(X,Y)
C...Return last x,y plotting location in user coordinates
      include 'pltlib.inc'
C
      X = X_LST
      Y = Y_LST
      return
      end


      subroutine GETCURSORXY(x,y,chkey)
C...Return current cursor (mouse) x,y location in user coordinates
C...when the mouse button or keyboard key is pressed.
C...chkey returns the character for the key pressed (if keyboard) 
      include 'pltlib.inc'
      character*1 chkey
C
      call getcursorxyabs(XA,YA,chkey)
C...get user coordinates
      x = XABS2usr(XA)
      y = YABS2usr(YA)
      return
      end


      subroutine GETCURSORXYABS(X,Y,chkey)
C...Return current cursor (mouse) X,Y location in absolute coordinates 
C...when the mouse button or keyboard key is pressed.
C...chkey returns the character for the key pressed (if keyboard) 
      include 'pltlib.inc'
      character*1 chkey
C
      call gw_curs(XZ,YZ,khar)
      chkey = char(khar)
      if(LGW_GEN) call gw_flush
C
C...get absolute coordinates
      X = X_ZM2ABS(XZ)
      Y = Y_ZM2ABS(YZ)
      return
      end


      subroutine GETCURSORXYC(x,y,ibtn)
C...Return current cursor (mouse) x,y location in user coordinates 
C...while mouse button is pressed.  Button status ibtn=1 (down), ibtn=0
C...(button released)
      include 'pltlib.inc'
      integer ibtn
C
      call getcursorxyabsc(XA,YA,ibtn)
C...get user coordinates
      x = XABS2usr(XA)
      y = YABS2usr(YA)
      return
      end


      subroutine GETCURSORXYABSC(X,Y,ibtn)
C...Return current cursor (mouse) X,Y location in absolute coordinates. 
C...Gets current mouse position when button is down (ibtn=1). Returns with
C...flag ibtn=0 when mouse button is released.
      include 'pltlib.inc'
      integer ibtn
C
      call gw_cursc(XZ,YZ,ibtn)
      if(LGW_GEN) call gw_flush
C
C...get absolute coordinates
      X = X_ZM2ABS(XZ)
      Y = Y_ZM2ABS(YZ)
      return
      end



      subroutine GETWINSIZE(XSIZE,YSIZE)
C...Returns current size of graphics window in absolute (page) units
      include 'pltlib.inc'
C
      XSIZE = float(GX_SIZ) / G_SCALE
      YSIZE = float(GY_SIZ) / G_SCALE
      return
      end

      subroutine GETPAGESIZE(XPAGE,YPAGE)
C...Returns current size of page in absolute (page) units
      include 'pltlib.inc'
C
      XPAGE = X_PAGE
      YPAGE = Y_PAGE
      return
      end
 
      subroutine GETREVVIDEO(lflag)
C...Gets reverse video flag
C   Returns   lflag = TRUE if reverse video is set
C
      include 'pltlib.inc'
      logical lflag
      lflag = LGW_REVVIDEO
      return
      end




      subroutine WINERASE
C...Erases the graphics area
      include 'pltlib.inc'
      if(LGW_GEN) call gw_clear
      return
      end


      subroutine PLFLUSH
C...Flush out plot components in buffers
      include 'pltlib.inc'
      if(LGW_GEN) call gw_flush
      if(LPS_GEN) call ps_flush
      return
      end
 

      subroutine DRAWTOSCREEN
C...Sets plotting destination to screen
      include 'pltlib.inc'
C
      call gw_drawtoscreen
      return
      end
 

      subroutine DRAWTOBUFFER
C...Sets plotting destination to background buffer
      include 'pltlib.inc'
C
      call gw_drawtobuffer
      return
      end


      subroutine SHOWBUFFER
C...Displays contents of background buffer to screen
      include 'pltlib.inc'
C
      call gw_showbuffer
      return
      end


      subroutine NEWCLIP(xmin,xmax,ymin,ymax)
C...Sets clip limits in user coordinates
      include 'pltlib.inc'
C
      X_MIN = xusr2ABS(xmin)
      X_MAX = xusr2ABS(xmax)
      Y_MIN = yusr2ABS(ymin)
      Y_MAX = yusr2ABS(ymax)
      call set_clip(X_MIN,Y_MIN,X_MAX,Y_MAX)
      call putprim(MinClipCommand,0,X_MIN,Y_MIN)
      call putprim(MaxClipCommand,0,X_MAX,Y_MAX)
C
      return
      end

      subroutine NEWCLIPABS(XMIN,XMAX,YMIN,YMAX)
C...Sets clip limits in absolute coordinates
      include 'pltlib.inc'
C
      call set_clip(XMIN,YMIN,XMAX,YMAX)
      call putprim(MinClipCommand,0,XMIN,YMIN)
      call putprim(MaxClipCommand,0,XMAX,YMAX)
C
      return
      end

      subroutine GETCLIP(xmin,xmax,ymin,ymax)
C...Returns clip limits in user coordinates
C
      include 'pltlib.inc'
C
      xmin = XABS2usr(CLP_XMIN)
      xmax = XABS2usr(CLP_XMAX)
      ymin = YABS2usr(CLP_YMIN)
      ymax = YABS2usr(CLP_YMAX)
      return
      end

      subroutine GETCLIPABS(XMIN,XMAX,YMIN,YMAX)
C...Returns clip limits specified in absolute (page) coordinates
C   i.e. in inches
C
      include 'pltlib.inc'
C
      XMIN = CLP_XMIN
      YMIN = CLP_YMIN
      XMAX = CLP_XMAX
      YMAX = CLP_YMAX
      return
      end

      subroutine CLRCLIP
C...Resets user clip limits to graphics window limits (no visible clipping)
c
      include 'pltlib.inc'
c
      call set_clip(0.0,0.0,X_PAGE,Y_PAGE)
      call putprim(MinClipCommand,0,0.0   ,   0.0)
      call putprim(MaxClipCommand,0,X_PAGE,Y_PAGE)
C
      return
      end



      subroutine GETZOOMABS(XOFF,YOFF,XFAC,YFAC)
C...Returns zoom offsets and scale factors
C     XOFF, YOFF are the offsets in absolute coordinates
C     XFAC, YFAC are the zoom factors applied to XY'=XYFAC*(XY+XYOFF) 
      include 'pltlib.inc'
C
      XOFF = XOFF_ZOOM
      YOFF = YOFF_ZOOM
      XFAC = XFAC_ZOOM
      YFAC = YFAC_ZOOM
C
      return
      end

      subroutine NEWZOOMABS(XOFF,YOFF,XFAC,YFAC)
C...Explicitly sets zoom offsets and scale factors
C   The parameters to NEWZOOMABS are the same as output from GETZOOMABS.
C     XOFF, YOFF are the offsets in absolute coordinates
C     XFAC, YFAC are the zoom factors applied to XY'=XYFAC*(XY+XYOFF) 
      include 'pltlib.inc'
C
      XOFF_ZOOM = XOFF
      YOFF_ZOOM = YOFF
      XFAC_ZOOM = XFAC
      YFAC_ZOOM = YFAC
c...Re-draw zoomed plot to X-window only
c     call REPLOT(1)
C
      return
      end

      subroutine USETZOOM(LXYSAME,LCURSOR)
C...User interactively sets zoom box, either by mouse selection, or
C   by asking for coordinates of the zoom rectangle
      logical LXYSAME,LCURSOR
      include 'pltlib.inc'
c
C...Get zoom parameters from user
      call set_zoom(XOFF_ZOOM,YOFF_ZOOM,XFAC_ZOOM,YFAC_ZOOM,
     &              LXYSAME,LCURSOR)
c...Re-draw zoomed plot to X-window only
c     call REPLOT(1)
      return
      end

      subroutine CLRZOOM
C...Resets zoom parameters to no-zoom condition
      include 'pltlib.inc'
      call NEWZOOMABS(0.,0.,1.,1.)
      return
      end




      function XABS2usr(X)
C...Converts absolute X to user x
      include 'pltlib.inc'
      XABS2usr = (X - X_ORG)/X_SCALE
      return
      end

      function YABS2usr(Y)
C...Converts absolute Y to user y 
      include 'pltlib.inc'
      YABS2usr = (Y - Y_ORG)/Y_SCALE
      return
      end

      function xusr2ABS(x)
C...Converts user x to absolute X 
      include 'pltlib.inc'
      xusr2ABS = x*X_SCALE + X_ORG
      return
      end

      function yusr2ABS(y)
C...Converts user y to absolute Y 
      include 'pltlib.inc'
      yusr2ABS = y*Y_SCALE + Y_ORG
      return
      end


 
      subroutine PLGRID (x,y,nx,xd,ny,yd,lmask)
C...Generates linear and non-linear grid patterns (with line masks)
C
C  Where: x,y  user coordinate of lower lefthand corner of
c              the grid to be generated.
c
c          nx  number of intervals in the x direction
c              if 'nx' is greater than 1000, then argument
c              'xd' will be treated as an array of interval values
c              with 'nx-1000' elements.  '-nx' indicates that the
c              actual vertical line generations are to be suppressed.
c          xd  (nx<1000)  user coordinate distance between uniformly 
c              spaced vertical lines
c              (nx>1000)  an array of values for spacing vertical
c              lines at varying intervals
c
c          ny  number of intervals in the y direction.  
c              if 'ny' is greater than 1000, then argument
c              'yd' will be treated as an array of interval values
c              with 'ny-1000' elements.  '-ny' indicates that the
c              actual horizontal line generations are to be suppressed.
c          yd  (ny<1000)  user coordinate distance between uniformly 
c              spaced horizontal lines
c              (ny>1000)  an array of values for spacing horizontal
c              lines at varying intervals
c
c       lmask  line mask bit pattern to be used in generating
c              the gridded form.
c
c  calls:   PLGRIDABS
C
      DIMENSION xd(*),yd(*)
      DIMENSION XDABS(500), YDABS(500)

C
      XABS = xusr2ABS(x)
      YABS = yusr2ABS(y)
      call GETFACTORS(xscale,yscale)
      XDABS(1) = xscale*xd(1)
      YDABS(1) = yscale*yd(1)
C
C...Decode grid interval information and scale arrays if necessary
      MX = IABS(nx)
      if(MX.GT.1000) then
        JX = MX/1000
        MX = MX - JX*1000
        do i=2, MX
          XDABS(i) = xscale*xd(i)
        end do
      ENDIF
      MY = IABS(ny)
      if(MY.GT.1000) then
        JY = MY/1000
        MY = MY - JY*1000
        do i=2, MY
          YDABS(i) = yscale*yd(i) 
        end do
      ENDIF
C
C...Call absolute coordinate routine
      call PLGRIDABS(XABS,YABS,nx,XDABS,ny,YDABS,lmask)
      RETURN
      END

 
      subroutine PLGRIDABS(X,Y,nx,XD,ny,YD,lmask)
C...Generates linear and non-linear grid patterns (with line masks)
C
C  Where: X,Y  absolute coordinate of lower lefthand corner of
c              the grid to be generated.
c
c          nx  number of intervals in the x direction
c              if 'nx' is greater than 1000, then argument
c              'xd' will be treated as an array of interval values
c              with 'nx-1000' elements.  '-nx' indicates that the
c              actual vertical line generations are to be suppressed.
c          XD  (nx<1000)  absolute coordinate distance between uniformly 
c              spaced vertical lines
c              (nx>1000)  an array of values for spacing vertical
c              lines at varying intervals
c
c          ny  number of intervals in the y direction.  
c              if 'ny' is greater than 1000, then argument
c              'yd' will be treated as an array of interval values
c              with 'ny-1000' elements.  '-ny' indicates that the
c              actual horizontal line generations are to be suppressed.
c          YD  (ny<1000)  absolute coordinate distance between uniformly 
c              spaced horizontal lines
c              (ny>1000)  an array of values for spacing horizontal
c              lines at varying intervals
c
c       lmask  line mask bit pattern to be used in generating
c              the gridded form.
c
c  calls:   PLOTABS
C
      DIMENSION XD(*),YD(*)
C
C...Decode grid interval information
      MX = IABS(nx)
      MY = IABS(ny)
      JX = MX/1000
      JY = MY/1000
      MX = MX - JX*1000
      MY = MY - JY*1000
C
C...Save and set line mask pattern
      LMSK = LMASK
      CALL GETPAT(IMASK)
      CALL NEWPAT(LMSK)
C
C...Set x ordinates for horizontal lines
      X1 = X
      X2 = X + XD(1)*FLOAT(MX)
C
C...Check for 'xd' single value or array
      IF (JX.NE.0) THEN
C...'XD' array, recompute right x ordinate
         X2 = X
         DO I=1,MX
           X2 = X2 + XD(I)
         END DO
        ENDIF
C
C...Generate horizontal lines
        Y2 = Y
        IF (NY.GT.0) THEN
          CALL PLOTABS(X1,Y2,+3)
          CALL PLOTABS(X2,Y2,+2)
        ENDIF
        J = 1
        DO I=1,MY
          Y2 = Y2 + YD(J)
          IF (NY.GT.0) THEN 
            CALL PLOTABS(X1,Y2,+3)
            CALL PLOTABS(X2,Y2,+2)
          ENDIF
          J = J + JY
        END DO
C
C...Generate vertical lines
       IF (NX.GT.0) THEN
         Y1 = y
         CALL PLOTABS(X1,Y1,+3)
         CALL PLOTABS(X1,Y2,+2)
         J = 1
         DO I=1,MX
           X1 = X1 + XD(J)
           CALL PLOTABS(X1,Y1,+3)
           CALL PLOTABS(X1,Y2,+2)
           J = J + JX
         END DO
       ENDIF
C
C...Restore line mask pattern
        CALL NEWPAT(IMASK)
C
      RETURN
      END
