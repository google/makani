C***********************************************************************
C    Module: plt_set.f 
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
C --- Xplot11 internal processing routines 
C
C     Version 4.46 11/28/01
C
C     Note:  These are plot-handling routines intended only
C            for internal use, including handling the replot buffer,
C            internal plot calls and clipping and zooming (the "set_xxx" 
C            routines).  Not intended for general consumption.
C***********************************************************************

      subroutine set_pen(ipen)
C...Sets current pen width in pixels
      include 'pltlib.inc'
      LST_PEN = ipen
      if(LGW_GEN) call gw_pen(ipen)
      if(LPS_GEN) call ps_pen(ipen)
      return
      end


      subroutine set_pat(ipat)
C...Sets current line pattern as integer bit pattern
      include 'pltlib.inc'
      LST_PAT = ipat
      if(LGW_GEN) call gw_linepattern(ipat)
      if(LPS_GEN) call ps_linepattern(ipat)
      return
      end


      subroutine set_color(icolor)
C...Sets foreground color for plotting
      include 'pltlib.inc'
      LST_CLR = icolor
      if(LGW_GEN) call gw_color(icolor)
      if(LPS_GEN) call ps_color(icolor)
      return
      end


      subroutine set_scl(xscale,yscale)
C...Sets user's plotting scale factors to xscale,yscale
      include 'pltlib.inc'
      X_SCALE = xscale
      Y_SCALE = yscale
      return
      end


      subroutine putprim(ityp,ival,xval,yval)
C...Installs plot primitive in save arrays
C   If the save arrays fill, the arrays are dumped to a temporary logfile
C
      include 'pltlib.inc'
      SAVE incore
C
      if(N_PRIM.EQ.0) then
        N_WRIT = 0
        incore = 0
      endif
C
C...Don't put primitives if at end of arrays and logging has been disabled
      if(N_PRIM.GT.Nstore_max .AND. .NOT.LPRIM_OPEN) return
C
C...Check number of primitives in in-core save arrays for full arrays
      if(incore.EQ.Nstore_max .AND. N_PRIM.NE.0) THEN
C
C...Primitives arrays are full, how about writing it out to logfile?
       if(.NOT.LPRIM_OPEN) then
C...Open new dump file, or overwrite existing one
         open(NPRIM_UNIT,file='xplot11_logfile',form='UNFORMATTED',
     &         status='UNKNOWN',err=10)
         rewind(NPRIM_UNIT)
         LPRIM_OPEN = .TRUE.
         N_WRIT = 0
       endif
C
C...Append in-core save arrays to disk file
C    first  record is number of elements in this save block, 
C    second record is integer type and integer value arrays, 
C    third  record is x,y value arrays
       write(NPRIM_UNIT) Nstore_max
       call wr_array(NPRIM_UNIT,Nstore_max,I_TYP,I_PRIM,X_PRIM,Y_PRIM)
       N_WRIT = N_WRIT + Nstore_max
C...incore index is rolled over to 0 and resumes counting up as index 
C   in the save arrays
       incore = 0
      endif
C  
      N_PRIM = N_PRIM  + 1
      incore = incore + 1
C
      I_TYP (incore) = ityp
      I_PRIM(incore) = ival
      X_PRIM(incore) = xval
      Y_PRIM(incore) = yval
C
      return
C
 10   write(*,*) '*** Open error on xplot11_logfile, logging disabled'
C...This will disable future logging (see first statement above)
      N_PRIM = Nstore_max + 1
      return
      end


      subroutine getprim(icount,ityp,ival,xval,yval)
C...Gets plot primitive from saved plot array or from log file
C
C   Input:  icount  integer giving primitive count
C                   icount<=0 as input indicates restart on prims
C                             list, rewind external prim file, if used
C   Output:         icount is incremented each fetch until the prims 
C                   list is exhausted, then icount is returned as -1 to 
C                   signal that no more prims are available
C
      include 'pltlib.inc'
      SAVE incore
C
      if(icount.LT.0) THEN
C...primitives-fetching is to be restarted from beginning
C
        if(LPRIM_OPEN) then
          if(N_WRIT .lt. N_PRIM) then
C...append rest of save arrays to disk, so arrays can be used as i/o buffers
            imax = N_PRIM - N_WRIT
            write(NPRIM_UNIT) imax 
            call wr_array(NPRIM_UNIT,imax,I_TYP,I_PRIM,X_PRIM,Y_PRIM)
            N_WRIT = N_WRIT + imax
          endif
          rewind(NPRIM_UNIT)
C...now read a buffer of stored prims
          read(NPRIM_UNIT,err=10) imax
          call rd_array(NPRIM_UNIT,imax,I_TYP,I_PRIM,X_PRIM,Y_PRIM)
         ELSE
C...No prim file used initialize incore to total # of prims
          imax = N_PRIM
        endif
        N_INCORE = imax
        incore = 0
        icount = 0
C
      elseif(icount.EQ.N_PRIM) THEN
C...no more primitives are available
        icount = -1
        return
      endif
C
C...Note:  icount  has the same meaning as N_PRIM in putprim
C...       incore  has the same meaning as incore in putprim
C
      if(incore.EQ.N_INCORE .AND. icount.ne.0) then
        if(LPRIM_OPEN) then
C...Read next chunk of primitives from file if opened
         read(NPRIM_UNIT,err=10) imax
         call rd_array(NPRIM_UNIT,imax,I_TYP,I_PRIM,X_PRIM,Y_PRIM)
         N_INCORE = imax
         incore = 0
       else
C...Shouldn't get to here if all went OK.
         write(*,*) 'Xplot11: Cannot read log file.'
         icount = -1
         return
        endif
      endif
C
C...Set the values for returning
      icount = icount  + 1
      incore = incore + 1
      ityp = I_TYP (incore)
      ival = I_PRIM(incore)
      xval = X_PRIM(incore)
      yval = Y_PRIM(incore)
C  
      return
C
 10   write(*,*) '? Xplot11: Error on reading log file.'
      return
      end


      subroutine rd_array(lunit,n,ityp,iprim,xprim,yprim)
c--- Fast unformatted reading of 2 integer and 2 real arrays from log file
      dimension ityp(n),  iprim(n),
     &          xprim(n), yprim(n)
      read(lunit,err=10) ityp,iprim
      read(lunit,err=10) xprim,yprim
      go to 20
 10   write(*,*) '? Xplot11: Error on reading log file.'
 20   return
      end


      subroutine wr_array(lunit,n,ityp,iprim,xprim,yprim)
c--- Fast unformatted writing of 2 integer and 2 real arrays to log file
      dimension ityp(n),  iprim(n),
     &          xprim(n), yprim(n)
      write(lunit) ityp,iprim
      write(lunit) xprim,yprim
      return
      end


      subroutine set_clip(XMIN,YMIN,XMAX,YMAX)
      include 'pltlib.inc'
C
C... clip to at least window limits
      CLP_XMIN = MAX( XMIN , 0.0 )
      CLP_YMIN = MAX( YMIN , 0.0 )
      CLP_XMAX = MIN( XMAX , X_PAGE )
      CLP_YMAX = MIN( YMAX , Y_PAGE )
C
      return
      end



      subroutine plot_1(X,Y,icode)	
C...Processing routine for internal plotting calls, absolute coordinates
C
C    X,Y    absolute coordinates
C    icode  function code (integer)
C
C          3   move to X,Y (move in absolute coordinates)
C          2   line to X,Y (line in absolute coordinates)
C
C         -2   line to X,Y (line and re-origin in absolute coordinates)
C         -3   move to X,Y (move and re-origin in absolute coordinates)
C
      include 'pltlib.inc'
      logical LCODE_OK
C
      icabs = abs(icode)
      LCODE_OK = (icabs.EQ.2  .OR. icabs.EQ.3)
C
      if(.NOT. LCODE_OK) then
        write(*,*) 'PLOTABS_1: unknown function code ',icode
        write(*,*) '       at point X,Y =',X,Y
        return
      endif 
C
C...use absolute coordinates to call device plotting
      X1 = X_LST
      Y1 = Y_LST
      X2 = X
      Y2 = Y
C
      if(icabs.eq.2) then
C...make copy of vector for zoomed clipping
        XX1 = X1
        YY1 = Y1
        XX2 = X2
        YY2 = Y2
C...clip vector to user plot limits (CLP_XMIN,CLP_XMAX,CLP_YMIN,CLP_YMAX)
        call clip_ls(XX1,YY1,XX2,YY2,ivis)
C
        if(ivis.NE.0) then
C...check for change in pen, line pattern or color
          if(I_PEN.NE.LST_PEN)  call set_pen(I_PEN)
          if(I_PAT.NE.LST_PAT)  call set_pat(I_PAT)
          if(I_CLR.NE.LST_CLR)  call set_color(I_CLR)
C
C...plot line segment in page coordinates
          if(LGW_GEN) call gw_line(XX1,YY1,XX2,YY2)
          if(LPS_GEN) call ps_line(XX1,YY1,XX2,YY2)
        endif
      endif
C
C...passed-in endpoint (absolute coordinates) now becomes "last" location
      X_LST = X2
      Y_LST = Y2
C
      if(icode.LT.0) then
C...re-origin
        X_ORG = X2
        Y_ORG = Y2
      endif
C     
      return
      end


      subroutine clip_ls(X1,Y1,X2,Y2,ivis)
C...Clips line segment against the clip window defined by 
C     CLP_XMIN,CLP_XMAX,CLP_YMIN,CLP_YMAX returning visibility flag ivis.
C      ivis=0 for no visible line segment 
C      ivis=1 for a  visible line segment
C
      include 'pltlib.inc'
      integer iclip_1
C
C... clip to zoomed clipping window or page limits
      CLPXMIN = MAX( X_ABS2ZM(CLP_XMIN) , 0.0 )
      CLPYMIN = MAX( Y_ABS2ZM(CLP_YMIN) , 0.0 )
      CLPXMAX = MIN( X_ABS2ZM(CLP_XMAX) , X_PAGE )
      CLPYMAX = MIN( Y_ABS2ZM(CLP_YMAX) , Y_PAGE )
C
C... zoomed coordinates for clipping
      X1 = X_ABS2ZM(X1)
      Y1 = Y_ABS2ZM(Y1)
      X2 = X_ABS2ZM(X2)
      Y2 = Y_ABS2ZM(Y2)
C
      ivis = 0
      if(iclip_1(X1,Y1,X2,Y2,CLPXMIN, 1.).eq.0) return
      if(iclip_1(Y1,X1,Y2,X2,CLPYMIN, 1.).eq.0) return
      if(iclip_1(X1,Y1,X2,Y2,CLPXMAX,-1.).eq.0) return
      if(iclip_1(Y1,X1,Y2,X2,CLPYMAX,-1.).eq.0) return
      ivis = 1
      return
      end


      integer function iclip_1(x1,y1,x2,y2,xlim,dir)
C...Basic line clipping, clips line segment against line x=xlim 
C   with visible side determined by dir (+1. or -1.)
C     dir=+1.  x>xlim is visible
C     dir=-1   x<xlim is visible
C
      iclip_1 = 0
      d1 = dir*(x1-xlim)
      d2 = dir*(x2-xlim)
      if(d1.EQ.0. .AND. d2.EQ.0.) return
C
      if(d1.GE.0. .OR.  d2.GE.0.) then
        iclip_1 = 1
        if(d1*d2 .LE. 0.) then
           dy = y2-y1
           d12 = d1-d2
           if(d1 .LT. 0.) then
             x1 = xlim
             y1 = y1 + dy*d1/d12
           endif
           if(d2 .LT. 0.) then
             x2 = xlim
             y2 = y2 + dy*d2/d12
           endif
        endif
      endif
      return
      end


      subroutine polyline_1(X,Y,n,ifill)
C...Processing routine for polyline calls, inputs in absolute coordinates
C    X,Y    coordinate arrays in absolute units
C    n      number of x,y points
C    ifill  fill flag, 0 for no fill, 1 for filled polygon
C
      include 'pltlib.inc'
      dimension X(n), Y(n)
      dimension XZ(MaxPolyLine),   YZ(MaxPolyLine)
      dimension XOUT(MaxPolyLine), YOUT(MaxPolyLine)
      dimension XCLP(5), YCLP(5)
C
C...For unfilled polyline treat this as a set of standard line segments
C   This avoids putting an unfilled polyline into the more complex filled
C   polyline clipper.
      if(ifill.LE.0) then
        if(n.gt.1) then
         call plot_1(X(1),Y(1),3)
         do i = 2, n
           call plot_1(X(i),Y(i),2)
         end do
        endif
        return
      endif
C
      if(n.GT.MaxPolyLine) then
        write(*,*) 'polyline_1: array overflow.  Increase MaxPolyline.'
        return
      endif
C
      if(n.LE.1) return
C
C...save last point for next draw command, just in case
      X_LST = X(n)
      Y_LST = Y(n)
C
C...convert to absolute, zoomed coordinates, check for bounding box
      XZ(1) = X_ABS2ZM(X(1))
      YZ(1) = Y_ABS2ZM(Y(1))
      xbbmin = XZ(1)
      ybbmin = YZ(1)
      xbbmax = xbbmin
      ybbmax = ybbmin
      do i=2, n
        XZ(i) = X_ABS2ZM(X(i))
        YZ(i) = Y_ABS2ZM(Y(i))
        IF(XZ(i).LT.xbbmin) xbbmin = XZ(i)
        IF(YZ(i).LT.ybbmin) ybbmin = YZ(i)
        IF(XZ(i).GT.xbbmax) xbbmax = XZ(i)
        IF(YZ(i).GT.ybbmax) ybbmax = YZ(i)
      end do
      NZ = n
C
C...Clip to zoomed clipping window or page limits
      CLPXMIN = MAX( X_ABS2ZM(CLP_XMIN) , 0.0 )
      CLPYMIN = MAX( Y_ABS2ZM(CLP_YMIN) , 0.0 )
      CLPXMAX = MIN( X_ABS2ZM(CLP_XMAX) , X_PAGE )
      CLPYMAX = MIN( Y_ABS2ZM(CLP_YMAX) , Y_PAGE )
C
C...Check for polygon outside clipping window, skip plotting
      if((xbbmin.GT.CLPXMAX) .OR.
     &   (xbbmax.LT.CLPXMIN) .OR.
     &   (ybbmin.GT.CLPYMAX) .OR.
     &   (ybbmax.LT.CLPYMIN) ) RETURN
C
C...Is polygon fully inside clipping window?
      if((xbbmax.GT.CLPXMAX) .OR.
     &   (xbbmin.LT.CLPXMIN) .OR.
     &   (xbbmax.GT.CLPYMAX) .OR.
     &   (ybbmin.LT.CLPYMIN) ) then
c
C...If polygon is not fully inside clipping window, clip it.
C   Get window vertices of clipping window assembled as a polygon,
C   these must be in clockwise order to clip inside the window
        XCLP(1) = CLPXMIN
        YCLP(1) = CLPYMIN
        XCLP(2) = CLPXMIN
        YCLP(2) = CLPYMAX
        XCLP(3) = CLPXMAX
        YCLP(3) = CLPYMAX
        XCLP(4) = CLPXMAX
        YCLP(4) = CLPYMIN
        XCLP(5) = CLPXMIN
        YCLP(5) = CLPYMIN
        nclp = 5
C...use special clipping for polylines that returns a clipped polyline
        call clip_poly0(XZ,YZ,NZ,XCLP,YCLP,nclp,XOUT,YOUT,nout)
c
C...Or if fully inside clip box, just plot the polyline as it is..
       else
        do i = 1, NZ
          XOUT(i) = XZ(i)
          YOUT(i) = YZ(i)
        end do
        nout = NZ
      endif
c
      if(nout.GT.1) then
C...check for change in pen, line pattern or color
        if(I_PEN.NE.LST_PEN)  call set_pen(I_PEN)
        if(I_PAT.NE.LST_PAT)  call set_pat(I_PAT)
        if(I_CLR.NE.LST_CLR)  call set_color(I_CLR)
C...plot polyline in page coordinates
        if(LGW_GEN) call gw_polyline(XOUT,YOUT,nout,ifill)
        if(LPS_GEN) call ps_polyline(XOUT,YOUT,nout,ifill)
      endif
      return
      end



      subroutine clip_poly0(xp,yp,np,xclp,yclp,nclp,xout,yout,nout)
C...Clips polyline xx,yy,nn against the clip window defined by 
C   xclp,yclp,nclp with output clipped polyline in xout,yout,nout
C
C   Clipping algorithm is a Sutherland-Hodgman clipper, implemented here
C   as a recursive (in Fortran!!) routine that clips each vertex in order
C   against each of the window edges.
C
      dimension xp(np),     yp(np), 
     &          xclp(nclp), yclp(nclp),
     &          xout(nout), yout(nout)
      dimension xfrst(5),   yfrst(5),
     &          xlst(5),    ylst(5),  ivis(5) 
C
      do n = 1, nclp
        ivis(n) = -1
      end do
      nout  = 0
      nlvl  = 0
      ilast = 0
c
C--- Feed the polyline points through the clipper point by point
      do n = 1, np
        x = xp(n)
        y = yp(n)
        call clip_poly1(x,y,ilast,xclp,yclp,nclp,xout,yout,nout,
     &                  xfrst,yfrst,xlst,ylst,ivis,nlvl)
      end do
C... Now do finish the clipping with a final call with last flag set
      ilast = 1
      call clip_poly1(x,y,ilast,xclp,yclp,nclp,xout,yout,nout,
     &                xfrst,yfrst,xlst,ylst,ivis,nlvl)
c  
      return
      end


      subroutine clip_poly1(x,y,ilast,xclp,yclp,nclp,xout,yout,nout,
     &                      xfrst,yfrst,xlst,ylst,ivis,nlvl)
C...Clipping routine for polylines clipped by polyline xclp,yclp,nclp
C   This routine is a recursive (via a faked-out call of clip_poly2)
C
      logical intsct
      dimension xclp(nclp), yclp(nclp),
     &          xout(nout), yout(nout)
      dimension xfrst(5), yfrst(5),
     &          xlst(5),  ylst(5),  ivis(5) 
C
C... Check for end of clipping process (ilast>0)
      if(nlvl.GE.nclp-1) THEN 
        if(ilast.LE.0) then
          nout = nout + 1
          xout(nout) = x
          yout(nout) = y
c          write(*,*) 'out ',nout,x,y
        endif
        return
      endif
C... Set clipping edge (same as recursion depth)
      nlvl = nlvl + 1
c
C... Check for closing flag (ilast=1)
C    Use first point as last vertex
      if(ilast.gt.0) then
        x = xfrst(nlvl)
        y = yfrst(nlvl)
      endif
c
C... Check point visibility
      x1 = xclp(nlvl)  
      y1 = yclp(nlvl)  
      dx = xclp(nlvl+1) - x1  
      dy = yclp(nlvl+1) - y1
      d2 = dy*(x-x1) - dx*(y-y1)
      ivisp = 0
      if(d2.GT.0.0)  ivisp = 1
c
C... Check for intersection, save first point in this level if ivis=-1
      intsct = .FALSE.
      if(ivis(nlvl).LT.0) then
        xfrst(nlvl) = x
        yfrst(nlvl) = y
        go to 20
      endif
      if (ivisp+ivis(nlvl).EQ.1) then
        d1 = dy*(xlst(nlvl)-x1) - dx*(ylst(nlvl)-y1)
        if((d1-d2).NE.0.) then
          intsct = .TRUE.
          frac = d1/(d1-d2)
          xint = xlst(nlvl) + frac*(x-xlst(nlvl))
          yint = ylst(nlvl) + frac*(y-ylst(nlvl))
        endif
      endif
c
C... Save point for next call to this level
C    If the close level flag is set (ilast=1) save it in ivis
   20 xlst(nlvl) = x
      ylst(nlvl) = y
      ivis(nlvl) = ivisp
      if(ilast.EQ.1) ivis(nlvl) = 2
c
C... Recurse to next level with intersection...
      if(intsct) then
        ilast = 0
        x = xint
        y = yint
        call clip_poly2(x,y,ilast,xclp,yclp,nclp,xout,yout,nout,
     &                  xfrst,yfrst,xlst,ylst,ivis,nlvl)
      endif
c
C... Call next level with vertex (if visible) or to close next level
      if(ivis(nlvl).GE.1) then
        x = xlst(nlvl)
        y = ylst(nlvl)
        if(ivis(nlvl).EQ.2) ilast = 1
        call clip_poly2(x,y,ilast,xclp,yclp,nclp,xout,yout,nout,
     &                  xfrst,yfrst,xlst,ylst,ivis,nlvl)
      endif
c
      nlvl = nlvl - 1
      return
      end


      subroutine clip_poly2(x,y,ilast,xclp,yclp,nclp,xout,yout,nout,
     &                      xfrst,yfrst,xlst,ylst,ivis,nlvl)
C...Dummy calling routine to allow recursion of clip_poly1
      dimension xclp(nclp), yclp(nclp),
     &          xout(nout), yout(nout)
      dimension xfrst(5), yfrst(5),
     &          xlst(5),  ylst(5),  ivis(5) 
C
      call clip_poly1(x,y,ilast,xclp,yclp,nclp,xout,yout,nout,
     &                xfrst,yfrst,xlst,ylst,ivis,nlvl)
      return
      end



      subroutine set_zoom(XOFF_ZOOM,YOFF_ZOOM,XFAC_ZOOM,YFAC_ZOOM,
     &                    LSAME,LCURS)
      logical LSAME, LCURS
      character chkey*1, line*80
C---------------------------------------------------------------
C     Sets new zoom parameters from cursor or keyboard input.
C
C     Input/   XOFF_ZOOM  zoom offsets
C     output:  YOFF_ZOOM
C              XFAC_ZOOM  zoom scaling factors
C              YFAC_ZOOM
C
C     Input:   LSAME   T if new zoom factors (XFAC,YFAC) must be the same 
C              LSAME   F if new zoom factor XFAC can be different than YFAC 
C              LCURS   T if input for zoom box comes from graphics input
C              LCURS   F if input for zoom box comes from keyboard
C---------------------------------------------------------------
C
      call GETWINSIZE(XSIZE,YSIZE)
      call GETCOLORINDEX(icolsave)
C--- Set zoom lines in default foreground color (black)
      icol = 1
C
      write(*,*)
      if(LCURS) then
       write(*,*) 'Mark off corners of blowup area'
       write(*,*) '(2 identical points default to current area)'
      else
       write(*,*) 'Enter x,y  coordinates of blowup area corners'
       write(*,*) '(2 identical points default to current area)'
       write(*,*) '(default is     user coords, use input "x y")'
       write(*,*) '(to specify absolute coords, use input "abs x y")'  
      endif
c
C...Get first point
      if(LCURS) then
       call GETCURSORXYABS(XABS1,YABS1,chkey)
       write(*,110) '1',XABS2usr(XABS1),YABS2usr(YABS1),XABS1,YABS1
      else
    1  write(*,*) 'point 1: '
       read(*,100,end=1) line
       if(line.eq.' ') then
         XABS1 = 0.
         YABS1 = 0.
        elseif(line(1:3).NE.'abs' .AND. line(1:3).NE.'ABS') then
         read(line,*,err=1) x1, y1
         XABS1 = xusr2ABS(x1)
         YABS1 = yusr2ABS(y1)
        else  
         read(line(4:80),*,err=1) XABS1, YABS1
       endif
      endif
      XZ = X_ABS2ZM(XABS1)
      YZ = Y_ABS2ZM(YABS1)
C... Use direct plotting calls to Xwindow to put crosshairs on screen
      call gw_color(icol)
      call gw_line(XZ, 0.0, XZ,    YSIZE)
      call gw_line(0.0, YZ, XSIZE, YZ)
      call gw_flush
c
C...Get second point
      if(LCURS) then
       call GETCURSORXYABS(XABS2,YABS2,chkey)
       write(*,110) '2',XABS2usr(XABS2),YABS2usr(YABS2),XABS2,YABS2
      else
    2  write(*,*) 'point 2: '
       read(*,100,end=2) line
       if(line.eq.' ') then
         XABS2 = 0.
         YABS2 = 0.
        elseif(line(1:3).NE.'abs' .AND. line(1:3).NE.'ABS') then
         read(line,*,err=2) x2, y2
         XABS2 = xusr2ABS(x2)
         YABS2 = yusr2ABS(y2)
        else
         read(line(4:80),*,err=2) XABS2, YABS2
       endif
      endif
      XZ = X_ABS2ZM(XABS2)
      YZ = Y_ABS2ZM(YABS2)
C... Use direct plotting calls to Xwindow to put crosshairs on screen
      call gw_line(XZ, 0.0, XZ,    YSIZE)
      call gw_line(0.0, YZ, XSIZE, YZ)
      call gw_flush
      call gw_color(icolsave)
C
C
C... Skip zooming stuff if points are the same
      if(XABS1.eq.XABS2 .and. YABS1.eq.YABS2) return
c
      XDIF = ABS(XABS2 - XABS1)
      YDIF = ABS(YABS2 - YABS1)
      if(XDIF.eq.0.0) XDIF = 0.0001*XSIZE
      if(YDIF.eq.0.0) YDIF = 0.0001*YSIZE
c
      XOFF_ZOOM = -MIN(XABS1,XABS2)
      YOFF_ZOOM = -MIN(YABS1,YABS2)
      XFAC_ZOOM = XSIZE/XDIF
      YFAC_ZOOM = YSIZE/YDIF
c
      if(LSAME) then
C... set equal x,y zoom factors
        fac = MIN(XFAC_ZOOM, YFAC_ZOOM)
        XFAC_ZOOM = fac
        YFAC_ZOOM = fac
c
C... re-center the zoom region
        XOFF_ZOOM = XOFF_ZOOM + 0.5*(XSIZE/fac-XDIF)
        YOFF_ZOOM = YOFF_ZOOM + 0.5*(YSIZE/fac-YDIF)
      endif
c
 100  format(a)
 110  format(' Pt ',a,2x,'usr x,y',2(2x,f11.6),3x,'abs X,Y',2(2x,f11.6))
      return
      end


C***Zoom transformation functions - to and from absolute<->zoomed

      function X_ABS2ZM(X)
C...Converts absolute X to zoomed X'
      include 'pltlib.inc'
      X_ABS2ZM = XFAC_ZOOM*(X + XOFF_ZOOM)
      return
      end

      function Y_ABS2ZM(Y)
C...Converts absolute Y to zoomed Y'
      include 'pltlib.inc'
      Y_ABS2ZM = YFAC_ZOOM*(Y + YOFF_ZOOM)
      return
      end



      function X_ZM2ABS(X)
C...Converts zoomed X' to absolute X
      include 'pltlib.inc'
      X_ZM2ABS = X/XFAC_ZOOM - XOFF_ZOOM
      return
      end


      function Y_ZM2ABS(Y)
C...Converts zoomed Y' to absolute Y
      include 'pltlib.inc'
      Y_ZM2ABS = Y/YFAC_ZOOM - YOFF_ZOOM
      return
      end








