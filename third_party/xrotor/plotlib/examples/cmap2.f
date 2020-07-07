C*********************************************************************** 
C    Module:  cmap.f
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


      program cmap2
c---------------------------------------------------------------
c     Color selection program.
c
c     Displays a 2-D slice through the R-G-B color space,
c     and gives the R,G,B components of a mouse-selected color.
c
c     The cutting plane is parallel to the RG, RB, or GB plane.
c     The position along the remaining B, G, or R axis is specified
c     by the % saturation 0..100.
c
c---------------------------------------------------------------
c
      character*2 plane
      character*1 axis, chkey
c
      character*40 colorname
c
      dimension x(5), y(5)
      logical lok
c
      ch = 0.03
c
 1000 format(a)
c
 5    write(*,1050) 
 1050 format(/' Enter cutting-plane orientation (RG, RB, or GB): ',$)
      read (*,1000) plane
c
      axis = ' '
      if(index('RGrg',plane(1:1)).NE.0 .AND.
     &   index('RGrg',plane(2:2)).NE.0       ) axis = 'B'
      if(index('RBrb',plane(1:1)).NE.0 .AND.
     &   index('RBrb',plane(2:2)).NE.0       ) axis = 'G'
      if(index('Gbgb',plane(1:1)).NE.0 .AND.
     &   index('Gbgb',plane(2:2)).NE.0       ) axis = 'R'
c
ccc   if(index('RGB',axis).EQ.0) go to 5
      if(index('RGB',axis).EQ.0) then
        call replot(4)
        call plot(0.,0.,+999)
        stop
      endif
c
c
      write(*,1100) axis
 1100 format( ' Enter % saturation along ',a1,' axis (0..100) : ', $)
      read (*,*) isat
c
      isat = max(  0,isat)
      isat = min( 99,isat)
c
      nc = 10
c
      sat = float(isat) / 100.0
c
c
c---- R,G,B unit vectors for projection onto x-y cutting plane
      xr = 0.
      yr = 0.
      zr = 0.
c
      xg = 0.
      yg = 0.
      zg = 0.
c
      xb = 0.
      yb = 0.
      zb = 0. 
c
      if(sat .lt. 0.50) then
c
        if(index('R',axis).EQ.1) then
          zr = 1.0
          xg = 1.0
          yb = 1.0
        endif
c
        if(index('G',axis).EQ.1) then
          yr = 1.0
          zg = 1.0
          xb = 1.0
        endif
c
        if(index('B',axis).EQ.1) then
          xr = 1.0
          yg = 1.0
          zb = 1.0
        endif
c
      else
c
        if(index('R',axis).EQ.1) then
          zr = 1.0
          yg = 1.0
          xb = 1.0
        endif
c
        if(index('G',axis).EQ.1) then
          xr = 1.0
          zg = 1.0
          yb = 1.0
        endif
c
        if(index('B',axis).EQ.1) then
          yr = 1.0
          xg = 1.0
          zb = 1.0
        endif
c
      endif
c
C
C---Initialize the plot package before we get into color plotting...
      CALL PLINITIALIZE
c
      call PLOPEN(0.8,0,1)
      call PLOT(5.5, 4.25, -3)
      call NEWFACTOR(6.0)
      call PLOT(-0.5,-0.5,-3)
c
c      call plopen(-0.8,0,5)
c      call plot(0.5,0.5,-3)
c      call newfactor(1.4)
c
      xdel = 1.0/float(nc)
      ydel = 1.0/float(nc)
c
      do 10 j = 1, nc
        y0 = ydel*float(j-1)
c
        do 105 i = 1, nc
          x0 = xdel*float(i-1)
c
          xx = x0 + 0.5*xdel
          yy = y0 + 0.5*ydel
          zz = sat
c
          r = xx*xr + yy*yr + zz*zr
          g = xx*xg + yy*yg + zz*zg
          b = xx*xb + yy*yb + zz*zb
c
          ir = int(256.0*r)
          ig = int(256.0*g)
          ib = int(256.0*b)
c
          x(1) = x0
          y(1) = y0
          x(2) = x0 + xdel
          y(2) = y0
          x(3) = x0 + xdel
          y(3) = y0 + ydel
          x(4) = x0
          y(4) = y0 + ydel
          x(5) = x0
          y(5) = y0
          n = 5
c
          call NEWCOLORRGB(ir,ig,ib)
          call POLYLINE(x,y,n,1)
c
 105    continue
 10   continue
c
      call PLFLUSH
c
      write(*,*)
      write(*,*) 'Click on colors...'
C
 200  call GETCURSORXY(xx,yy,chkey)

c      ikey = ichar(chkey)
c      write(*,*) ikey

      zz = sat
c
      r = xx*xr + yy*yr + zz*zr
      g = xx*xg + yy*yg + zz*zg
      b = xx*xb + yy*yb + zz*zb
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
      go to 5
      end



      logical function lok(ir,ig,ib)
      lok = ir.LE.255 .AND. ig.LE.255 .AND. ib.LE.255  .AND.
     &      ir.GE.0   .AND. ig.GE.0   .AND. ib.GE.0  
      return
      end

