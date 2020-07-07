C*********************************************************************** 
C    Module:  defmap.f
C 
C    Copyright (C) 1996 Harold Youngren, Mark Drela 
C 
C    This program is free software; you can redistribute it and/or modify 
C    it under the terms of the GNU General Public License as published by 
C    the Free Software Foundation; either version 2 of the License, or 
C    (at your option) any later version. 
C 
C    This program is distributed in the hope that it will be useful, 
C    but WITHOUT ANY WARRANTY; without even the implied warranty of 
C    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
C    GNU General Public License for more details. 
C 
C    You should have received a copy of the GNU General Public License 
C    along with this program; if not, write to the Free Software 
C    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA. 
C
C    Report problems to:    guppy@maine.com 
C                        or drela@mit.edu  
C*********************************************************************** 


      program defmap
C
C   Displays Default colormap in a bar
C
C
      dimension xp(100), yp(100), x(100), y(100)
C
      character*24 colorname, inp*2
C
      CH = 0.125
C
      PI = 4.0*ATAN(1.0)
C
C---Decide about what devices to plot to
      WRITE(*,*) ' '
   1  WRITE(*,*) ' Enter -1 for no PS, 0 for B&W PS, 1 for color PS'
      READ(*,1000,end=2000) INP
      ips = -1
      if(INP.ne.' ') then
        READ(INP,*,end=2000,err=2000) ips
      endif
      IDEV = 1
      IF(ips.eq.0) IDEV = 3
      IF(ips.ge.1) IDEV = 5
      ipslu = 0
C
C---- for REPLOT:  X11 only
      IDEVRP = 1
      CALL PLINITIALIZE
C
C---Now, how many colors...
      ncolors = 10
C
      CALL PLOPEN(0.7,ipslu,IDEV)
      CALL PLOTABS(1.5,1.0,-3)
c
      call GETCOLOR(ICOL0)
C
c---- plot bar
      dx = 1.0
      dy = 5.0/float(ncolors)
      do ii = 1,ncolors
        call NEWCOLOR(ii)
c
        x0 = 0.0
        y0 = dy*float(ii-1)
c
        xp(1) = x0
        yp(1) = y0
        xp(2) = x0+dx
        yp(2) = y0
        xp(3) = x0+dx
        yp(3) = y0+dy
        xp(4) = x0
        yp(4) = y0+dy
        xp(5) = xp(1)
        yp(5) = yp(1)
        call POLYLINE(xp,yp,5,1)
C
        call GETCOLORRGB(ii,ired,igrn,iblu,colorname)
C
        xplt =      xp(2)        + 3.0*ch
        yplt = 0.5*(yp(2)+yp(3)) - 0.5*ch
        call plnumb(xplt,yplt,ch,float(ii),0.0,-1)
C
        xplt = xplt + 6.0*ch
        call plchar(xplt,yplt,ch,colorname,0.0,24)
C
        xplt = xplt + 26.0*ch
        call NEWCOLORRGB(255,0,0)
        call plnumb(xplt,yplt,ch,float(ired),0.0,-1)
C
        xplt = xplt + 4.0*ch
        call NEWCOLORRGB(0,255,0)
        call plnumb(xplt,yplt,ch,float(igrn),0.0,-1)
C
        xplt = xplt + 4.0*ch
        call NEWCOLORRGB(0,0,255)
        call plnumb(xplt,yplt,ch,float(iblu),0.0,-1)
c
      end do
C
      call NEWCOLOR(ICOL0)
      xplt = 0.0
      yplt = yplt + dy + 2.0*ch
      call plchar(xplt,yplt,1.5*ch,'Default Colormap',0.0,16)
C
      CALL PLFLUSH
C
      WRITE(*,*) 'Hit return to end test'
      READ(5,1000) DUMMY
 1000 FORMAT(A)
C
C      GO TO 1
C
 2000 CALL PLOT(0.0,0.0,+999)
      STOP
      END











