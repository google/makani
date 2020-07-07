C*********************************************************************** 
C    Module:  contest.f
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

      program contest
C
C--- Test of Xplot11 quadrilateral and triangle contour primitives
C
      CHARACTER CHR*2, HUES*10
      DIMENSION X(4), Y(4), Z(4)
      DIMENSION XTRI(3), YTRI(3), ZTRI(3)
C
      DIMENSION XCU(50), YCU(50)
      DIMENSION XCL(50), YCL(50)
      DIMENSION XP(50), YP(50), NE(50)
C
      DATA X / 0., 1., 1., 0. /
      DATA Y / 0., 0., 1., 1. /
      DATA Z / 0., 1., 0., 2. /
C
      IDEV = 3
      nlevel  = 50
      ncolors = 64
      WRITE(*,*) ' '
      WRITE(*,*) 'Contour primitives test:'
      WRITE(*,*) '  (contour fills on single square polygon)'
      WRITE(*,*) ' ' 
      WRITE(*,*) 'Data points...'
      do i = 1, 4
        write(*,*) i,'   x = ',X(i),' y = ',Y(i),' z = ',Z(i)
      end do
C
C---Decide about what devices to plot to
      WRITE(*,*) ' '
   1  WRITE(*,*) ' Enter -1 for no PS, 0 for B&W PS, 1 for color PS'
      READ(*,1000,end=2000) CHR
      ips = -1
      if(CHR.ne.' ') then
        READ(CHR,*,end=2000,err=2000) ips
      endif
      IDEV = 1
      IF(ips.eq.0) IDEV = 3
      IF(ips.ge.1) IDEV = 5
      ipslu = 0
C
C--- Get contour data
      ZL = 0.
      ZU = 2. 
      write(*,*) ' '
      write(*,*) 'Contour limits ',ZL,' to ',ZU
      write(*,*) 'Enter # of contour levels'
      read (*,*) nlevel
C
      ipslu = 0
      CALL PLINITIALIZE
      HUES = 'ROYGCBM'
C
C---Set up colormap spectrum colors
      if(ncolors.LE.1) ncolors = 0
      CALL COLORSPECTRUMHUES(ncolors,HUES)
C
      do ITYP = 1, 2
C
       CALL PLOPEN(0.5,ipslu,IDEV)
      
       call newcolorname('green')
       CALL PLOTABS(.75,.75,-3)
       CALL PLCHAR (999.,999.,.1,'Contour test ',0.,-1)
       if(ITYP.NE.2) then
         WRITE(*,*) 'Polygon contoured and filled as quadrilateral'
         CALL PLCHAR (999.,999.,.1,'Quadrilateral',0.,-1)
        else
         WRITE(*,*) 'Polygon contoured and filled as two triangles'
         CALL PLCHAR (999.,999.,.1,'Two Triangles',0.,-1)
       endif
       CALL PLOT(0.,-0.5,-3)
       CALL PLCHAR (999.,999.,.1,'Nlevels = ',0.,-1)
       CALL PLNUMB (999.,999.,.1,FLOAT(nlevel),0.,-1)
       CALL PLCHAR (999.,999.,.1,'  Ncolors = ',0.,-1)
       CALL PLNUMB (999.,999.,.1,FLOAT(ncolors),0.,-1)
       CALL PLOT(2.,2.,-3)
       call factor(4.)
C
C--- Set contour levels and increments
       NCONT = NLEVEL + 1
       DZ = (ZU-ZL)/FLOAT(NCONT)
C
       DO N = 1, NCONT
         ZUPR = FLOAT(N)*DZ
         ZLWR = FLOAT(N-1)*DZ
C
C--- Set color based on contour #
         ICOL = (NCOLORS-1)*FLOAT(N-1)/FLOAT(NCONT-1) + 1
         CALL NEWCOLOR(-ICOL)
C
C--- Reset the line and area counters for each level
         NA = 0
         NV = 0
         NCU = 0
         NCL = 0 
C
         if(ITYP.NE.2) then
C
C--- Contour a quadrilateral
          CALL CONTQUAD(X,Y,Z,ZUPR,ZLWR,
     &              NCU,XCU,YCU,
     &              NCL,XCL,YCL,
     &              NA,NE,NV,XP,YP)
C
         else
C
C--- Triangle contouring, use two triangles, split quad on 1-3 diagonal
          xtri(1) = x(1)
          ytri(1) = y(1)
          ztri(1) = z(1)
          xtri(2) = x(2)
          ytri(2) = y(2)
          ztri(2) = z(2)
          xtri(3) = x(3)
          ytri(3) = y(3)
          ztri(3) = z(3)
          CALL CONTTRI(xtri,ytri,ztri,ZUPR,ZLWR,
     &                  NCU,XCU,YCU,
     &                  NCL,XCL,YCL,
     &                  NA,NE,NV,XP,YP)
          xtri(1) = x(3)
          ytri(1) = y(3)
          ztri(1) = z(3)
          xtri(2) = x(4)
          ytri(2) = y(4)
          ztri(2) = z(4)
          xtri(3) = x(1)
          ytri(3) = y(1)
          ztri(3) = z(1)
          CALL CONTTRI(xtri,ytri,ztri,ZUPR,ZLWR,
     &                  NCU,XCU,YCU,
     &                  NCL,XCL,YCL,
     &                  NA,NE,NV,XP,YP)
         endif
C
C
C--- Plot the filled contour polygons
         nv = 1
         DO IA = 1, NA
           call polyline(xp(nv),yp(nv),ne(ia),1)
           nv = nv+ne(ia)
         END DO
C
C--- Plot the contour lines (w/o color in this case). 
C    Otherwise you could leave out the polygon fills and comment out the 
C    color change to BLACK to get colored line contours.
C
         call newcolorname('BLACK')
C--- All lower contour lines
         do nn = 1, ncl,2
           call plot(xcl(nn),ycl(nn),3)
           call plot(xcl(nn+1),ycl(nn+1),2)
         end do
C--- And the last upper line
         if(N.EQ.NCONT) then
          do nn = 1, ncu,2
           call plot(xcu(nn),ycu(nn),3)
           call plot(xcu(nn+1),ycu(nn+1),2)
          end do
         endif
C
       END DO
       CALL PLFLUSH
C
       read(*,1000) chr
       CALL PLOT(0.,0.,-999)
C
      end do
      CALL PLOT(0.,0.,+999)
C
 1000 FORMAT(A)
C
 2000 STOP
      END














