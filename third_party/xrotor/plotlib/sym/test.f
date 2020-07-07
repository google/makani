C*********************************************************************** 
C    Module:  test.f  (test routine for fonts in Xplot/sym)
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


      PROGRAM TEST
C
      IDEV = 3
      IPSLU = 0
      SIZE = 7.0
      IPEN = 3
      CH = 0.020
C
cc      size = 5.0
cc      ipen = 2
C
      CALL PLINITIALIZE
      CALL PLOPEN(-0.95,IPSLU,IDEV)
      CALL NEWFACTOR(SIZE)
      CALL NEWPEN(IPEN)
C
      CALL PLOTABS(0.60,1.20,-3)
C
      WRITE(*,*) 'Plotting PLCHAR set...'
      CALL PLCHAR(0.0,1.25,CH,
     & 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz',0.0,52)
      CALL PLCHAR(0.0,1.20,CH,
     & '0123456789,.;:`"!?@#$%&|()[]{}<>_+-*=/^~            ',0.0,52)
C
      WRITE(*,*) 'Plotting PLSLAN set...'
      CALL PLSLAN(0.0,1.10,CH,
     & 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz',0.0,52)
      CALL PLSLAN(0.0,1.05,CH,
     & '0123456789,.;:`"!?@#$%&|()[]{}<>_+-*=/^~            ',0.0,52)
C
      WRITE(*,*) 'Plotting PLMATH set...'
      CALL PLMATH(0.0,0.95,CH,
     & 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz',0.0,52)
      CALL PLMATH(0.0,0.90,CH,
     & '0123456789,.;:`"!?@#$%&|()[]{}<>_+-*=/^~            ',0.0,52)
C
      WRITE(*,*) 'Plotting PLSYMB set...'
      DO IS=0, 13
        XX = 0.05*FLOAT(IS)
        CALL PLSYMB(XX,0.80,CH,IS,0.0,0)
      ENDDO
C
      WRITE(*,*) 'Plotting sample character strings...'
C
      CALL PLMATH(0.0 ,0.55,CH,' a2+b2=g2',0.0,9)
C
      CALL PLMATH(0.0 ,0.45,CH,'  2  2  2',0.0,9)
      CALL PLCHAR(0.0 ,0.45,CH,' a +b =c ',0.0,9)
C
      CALL PLMATH(0.0 ,0.35+0.15*CH,
     &                      CH,'R_____   ',0.0,9)
      CALL PLMATH(0.0 ,0.35,CH,'  2  2   ',0.0,9)
      CALL PLSLAN(0.0 ,0.35,CH,' a +b =c ',0.0,9)
C
      CALL PLMATH(0.30,0.55,CH,'    2',30.0,5)
      CALL PLCHAR(0.30,0.55,CH,'E=mc ',30.0,5)
C
C
      CALL PLMATH(0.30,0.45,CH,'F&= &   &   & ',-30.0,14)
      CALL PLCHAR(0.30,0.45,CH,'   u x+v y+w z',-30.0,14)
C
      CALL PLMATH(0.50,0.55,CH,'l-     ',0.0,7)
      CALL PLCHAR(0.50,0.55,CH,'  shock',0.0,7)
C
      CALL PLMATH(0.50,0.45,CH,'>=Nf',0.0,4)
      CALL PLCHAR(0.50,0.45,CH,'u   ',0.0,4)
C
      CALL PLMATH(0.75,0.55,CH,'V= n  H',0.0,7)
      CALL PLCHAR(0.75,0.55,CH,'  (  ) ',0.0,7)
      CALL PLSLAN(0.75,0.55,CH,'    t  ',0.0,7)
C
      CALL PLMATH(0.75,0.45,CH,'   __',0.0,5)
      CALL PLMATH(0.75,0.45,CH,'V=Rn',0.0,4)
      CALL PLSLAN(999.0,999.0,CH,'t',0.0,1)
C
      CALL PLMATH(0.75,0.35,CH,' =   M ',0.0,7)
      CALL PLCHAR(0.75,0.35,CH,'x [A] b',0.0,7)
C
      CALL PLMATH(0.0-CH,0.25-0.4*CH,2.0*CH,'I',0.0,1)
      CALL PLMATH(0.0,0.25,CH,'  ( )  ',0.0,7)
      CALL PLCHAR(0.0,0.25,CH,' F x dx',0.0,7)
C
      CALL PLMATH(0.0,0.15,CH,'e{ ',0.0,3)
      CALL PLCHAR(0.0,0.15,CH,'  1',0.0,3)
C
      CALL PLCHAR(0.25,0.25,CH,'273 K',0.0,5)
      CALL PLMATH(0.25,0.25,CH,'   " ',0.0,5)
C
      CALL PLSLAN(0.50,0.25,CH,'       y',0.0,8)
      CALL PLMATH(0.50,0.25,CH,'g=-$G/$ ',0.0,8)
C
      CALL PLCHAR(0.75,0.25,CH,'  tan    ',0.0,9)
      CALL PLSLAN(0.75,0.25,CH,'      y/x',0.0,9)
      CALL PLMATH(0.75,0.25,CH,'q=   M   ',0.0,9)
C
      CALL PLCHAR(0.25,0.15,CH,'Underline',0.0,9)
      CALL PLCHAR(0.25,0.15,CH,'_________',0.0,9)
C
      CALL PLSLAN(0.50,0.15,CH,'Overline',0.0,8)
      CALL PLMATH(0.50,0.15,CH,'________',0.0,8)
C
      CALL PLSLAN(0.75,0.15,CH,'  r=0',0.0,5)
      CALL PLMATH(0.75,0.15,CH,'q#   ',0.0,5)
      CALL PLMATH(0.75,0.15,CH,'  ^  ',0.0,5)
      CALL PLMATH(0.75,0.15+0.4*CH,
     &                      CH,'^    ',0.0,5)
C
      CALL PLMATH(0.00,0.05+1.2*CH,
     &                      CH,'  *       ',0.0,10)
      CALL PLMATH(0.00,0.05+0.4*CH,
     &                      CH,'  >    >  ',0.0,10)
      CALL PLMATH(0.00,0.05,CH,'>   >    >',0.0,10)
      CALL PLCHAR(0.00,0.05,CH,'a=  r+2  v',0.0,10)
      CALL PLMATH(0.00,0.05,CH,'  W#   W# ',0.0,10)
C
      CALL PLSLAN(0.30,0.05,CH,'       ~ ',0.0,9)
      CALL PLCHAR(0.30,0.05,CH,'ln(1+ )  ',0.0,9)
      CALL PLMATH(0.30,0.05,CH,'     v  v',0.0,9)
C
      CALL PLSLAN(0.55,0.05,CH,'        ~     ',0.0,14)
      CALL PLCHAR(0.55,0.05,CH,' {g x }   g  x',0.0,14)
      CALL PLMATH(0.55,0.05,CH,'O  ( )     `O ',0.0,14)
C
      CALL PLFLUSH
C
ccc      call usetzoom(.true.,.true.)
ccc      call replot(1)
C
      READ(*,1) DUMMY
 1    FORMAT(A)
C
      CALL PLOT(0.0,0.0,+999)
C
      END

