C*********************************************************************** 
C    Module:  zoomtest.f
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


      PROGRAM ZOOMTEST
C
C---- dot-pattern masks for use with PLGRID, NEWPAT, etc.
C
C     mask0:  _________________________   (solid)
C         1:  .........................
C         2:  . . . . . . . . . . . . .
C         3:  .   .   .   .   .   .   .
C         4:  .       .       .       .
C
      CHARACTER LINE*80
      LOGICAL LXYSAME, LCURSOR
      data  mask0,  mask1,  mask2,  mask3,  mask4
     &   / -1    , -21846, -30584, -32640, -32768 /


C
      IDEV = 1
      IPSLU = 0
      SIZE = 0.8
C
      CH = 0.020
C
      CALL PLINITIALIZE
      CALL PLOPEN(0.78,IPSLU,IDEV)
      CALL DRAWTOBUFFER
      CALL NEWFACTOR(SIZE)
C
      CALL PLOTABS(1.00,1.00,-3)
C
      call PLGRID(0.0,0.0,22,0.5,17,0.5,MASK2)
      call PLGRID(0.0,0.0,11,1.0, 9,1.0,MASK1)
      call PLGRID(0.0,0.0, 3,5.0, 2,5.0,MASK0)
      call PLSLAN(0.1,0.1,0.2,'abcDEF123',0.0,9)
      CALL NEWCOLORNAME('green')
      call PLCHAR(1.1,1.1,0.2,'321GHIjkl',0.0,9)
      CALL NEWCOLORNAME('red')
      call PLSLAN(2.1,2.1,0.2,'mnoPQR123',0.0,9)
      CALL NEWCOLORNAME('blue')
      call PLCHAR(3.1,3.1,0.2,'321STUvwx',0.0,9)
      CALL NEWCOLORNAME('black')
c
      CALL PLFLUSH
c
      write(*,*) ' '
      write(*,*) 'The zoom can be done with same X,Y scales'
      write(*,*) 'Either mouse or keyboard can set the zoom rectangle'
      write(*,*) 'Enter XYsame, MouseInput flags (T/F):'
      LXYSAME = .TRUE.
      LCURSOR = .TRUE.
      read(*,1000) LINE
      if(LINE.NE.' ') THEN      
        read(LINE,*,err=10) LXYSAME,LCURSOR
      endif
 10   write(*,*) ' '
C
      do k=1, 3
        call USETZOOM( LXYSAME , LCURSOR )
        call REPLOT(1)
      enddo
      pause
      call CLRZOOM
      call REPLOT(1)
      pause
c
      CALL PLOT(0.0,0.0,+999)
 1000 FORMAT(A)
C
      END






























