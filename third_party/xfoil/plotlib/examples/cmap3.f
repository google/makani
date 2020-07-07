C*********************************************************************** 
C    Module:  cmap3.f
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


      program cmap3
c---------------------------------------------------------------
c     Color selection program.
c
c     Displays a 2-D slice through the R-G-B color space,
c     and gives the R,G,B components of a mouse-selected color.
c
c     The cutting plane is perpendicular to the diagonal
c     axis running from  R,G,B = 0,0,0 (black),  to 
c     R,G,B = 1,1,1 (white).  The position of this cutting 
c     plane is specified by the % saturation 0..300.  
c     The plane passes through one or three pure-color 
c     corners for the specific saturations shown.
c
c       0%   (0 0 0)                     black
c     100%   (1 0 0), (0 1 0), (0 0 1)   red   , green  , blue 
c     200%   (1 1 0), (1 0 1), (0 1 1)   yellow, magenta, cyan
c     300%   (1 1 1)                     white
c
c     For 0-100% and 200-300%, the cutting plane is a triangle.
c     For 100-200%, the plane is a hexagon.
c
c---------------------------------------------------------------
c
      dimension x(4), y(4)
      logical lok
      character*1 chkey
c
      ch = 0.03
      call PLINITIALIZE
c
 1    continue
c
      write(*,1100)
 1100 format(/' Enter % saturation (0..300) : ', $)
      read (*,*) isat
c
      if(isat.eq.0) go to 500
c
      isat = max(  1,isat)
      isat = min(299,isat)
c
      nc1 = 10
c
      nc = nc1
      if(isat.gt.100) nc = (nc1*isat)/ 100     
      if(isat.gt.200) nc = (nc1*200 )/(300-isat) + 1
c
      sat = float(isat) / 100.0
c
c
c---- R,G,B unit vectors for projection onto x-y cutting plane
      xr = -sat
      yr = -sat/sqrt(3.0)
c
      xg =  0.0
      yg =  sat*2.0/sqrt(3.0)
c
      xb =  sat
      yb = -sat/sqrt(3.0)
c
      call COLORMAPDEFAULT
c
      call PLOPEN(0.8,0,1)
c
      call PLOT(5.5, 4.25, -3)
      call NEWFACTOR(3.0)
c
c
      area = sat * 2.0*sqrt(3.0)
c
      xdel = sat*     2.0 /float(nc)
      ydel = sat*sqrt(3.0)/float(nc)
c
      do 10 j = 1, nc
        y0 = yr + ydel*float(j-1)
c
        do 105 i = 1, nc-j+1
          x0 = xr + xdel*(float(i-1) + 0.5*float(j-1))
c
          xx = x0 + 0.5*xdel
          yy = y0 +     ydel/3.0
c
          r = ((xg-xb)*(yy-yb) - (yg-yb)*(xx-xb))/area
          g = ((xb-xr)*(yy-yr) - (yb-yr)*(xx-xr))/area
          b = ((xr-xg)*(yy-yg) - (yr-yg)*(xx-xg))/area
c
          ir = int(256.0*r)
          ig = int(256.0*g)
          ib = int(256.0*b)
c
          if( lok(ir,ig,ib) ) then
            x(1) = x0
            y(1) = y0
            x(2) = x0 + xdel
            y(2) = y0
            x(3) = x0 + xdel*0.5
            y(3) = y0 + ydel
            x(4) = x0
            y(4) = y0
            n = 4
c
            call NEWCOLORRGB(ir,ig,ib)
            call POLYLINE(x,y,n,1)
          endif
c
c
          if(i.eq.nc-j+1) go to 105

          xx   = x0 +     xdel
          yy   = y0 + 2.0*ydel/3.0
c
          r = ((xg-xb)*(yy-yb) - (yg-yb)*(xx-xb))/area
          g = ((xb-xr)*(yy-yr) - (yb-yr)*(xx-xr))/area
          b = ((xr-xg)*(yy-yg) - (yr-yg)*(xx-xg))/area
c
          ir = int(256.0*r)
          ig = int(256.0*g)
          ib = int(256.0*b)
c
          if( lok(ir,ig,ib) ) then
            x(1) = x0 + xdel
            y(1) = y0
            x(2) = x0 + xdel*1.5
            y(2) = y0 + ydel
            x(3) = x0 + xdel*0.5
            y(3) = y0 + ydel
            x(4) = x0 + xdel
            y(4) = y0
            n = 4
c
            call NEWCOLORRGB(ir,ig,ib)
            call POLYLINE(x,y,n,1)
          endif
c
 105    continue
 10   continue
c
      call PLFLUSH
c
      write(*,*) 'Click on colors...'
C
 200  call GETCURSORXY(xx,yy,chkey)
c
      r = ((xg-xb)*(yy-yb) - (yg-yb)*(xx-xb))/area
      g = ((xb-xr)*(yy-yr) - (yb-yr)*(xx-xr))/area
      b = ((xr-xg)*(yy-yg) - (yr-yg)*(xx-xg))/area
c
      ir = int(256.0*r)
      ig = int(256.0*g)
      ib = int(256.0*b)
c
      write(*,1500) ir, ig, ib
 1500 format(1x,'R G B  = ', i4,',',i4,',',i4)
c
      if( lok(ir,ig,ib) ) then
        go to 200
      endif
c
      go to 1
c
 500  call PLOT(0.0,0.0,+999)
      stop
C
      end



      logical function lok(ir,ig,ib)
      lok = ir.LE.255 .AND. ig.LE.255 .AND. ib.LE.255  .AND.
     &      ir.GE.0   .AND. ig.GE.0   .AND. ib.GE.0  
      return
      end

