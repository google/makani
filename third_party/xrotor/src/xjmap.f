C***********************************************************************
C    Module:  xjmap.f
C 
C    Copyright (C) 2011 Mark Drela 
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
C***********************************************************************

      SUBROUTINE JMAP
      INCLUDE 'XROTOR.INC'
C----------------------------------------------------------
C     Generates and dumps data for an operating map of 
C     Cp vs J with contours of efficiency and blade angle.
C----------------------------------------------------------
C
      PARAMETER (LX=91,KX=81)
      DIMENSION EF(LX,KX), BE(LX,KX)
      DIMENSION RJ(LX), CP(KX)
      INTEGER KLOW(LX), KUPP(LX)
C
      DIMENSION BETSAV(IX)
      LOGICAL LABCON
C
      GREEK = .FALSE.
C
      CALL ASKR('Enter smallest J (advance ratio * pi) ^',RJ1)
      CALL ASKR('Enter  largest J (advance ratio * pi) ^',RJ2)
      CALL ASKR('Enter J increment ^',RJINC)
      IF(RJINC.EQ.0.0) RETURN
C
      NRJ = INT( (RJ2-RJ1)/RJINC + 1.01 )
      IF(NRJ.GT.LX) THEN
       WRITE(*,*)
       WRITE(*,*) 'Array overflow.'
       WRITE(*,*) 'Number of J`s set to', LX
       NRJ = LX
      ENDIF
C
      CALL ASKR('Enter smallest Cp^',CP1)
      CALL ASKR('Enter  largest Cp^',CP2)
      CALL ASKR('Enter Cp increment ^',CPINC)
      IF(CPINC.EQ.0.0) RETURN
C
      NCP = INT( (CP2-CP1)/CPINC + 1.01 )
      IF(NCP.GT.KX) THEN
       WRITE(*,*)
       WRITE(*,*) 'Array overflow.'
       WRITE(*,*) 'Number of Cp`s set to', KX
       NCP = KX
      ENDIF
C
      CALL ASKS('Enter dump filename^',FNAME)
C
C---- save current V, advance ratio, and blade angle for restoration
      VELSAV = VEL
      ADVSAV = ADV
      DO I=1, II
        BETSAV(I) = BETA(I)
      ENDDO
C
C---- set specifed advance ratios and Cp`s
      DO L=1, NRJ
        RJ(L) = RJ1 + RJINC*FLOAT(L-1)
      ENDDO
      DO K=1, NCP
        CP(K) = CP1 + CPINC*FLOAT(K-1)
      ENDDO
C
      WR = VEL/ADV
      RE6 = RHO*WR*RAD/RMU * 1.0E-6
      AMACH = WR/VSO
C
      WRITE(*,1300) RE6, AMACH
 1300 FORMAT(
     &  / 1X, 'Tip speed parameters to be held fixed',
     &          '(from current conditions):'
     &  / 1X, 'Reynolds number   (wR) R / nu =', F6.3,' e 6' 
     &  / 1X, 'Mach number           wR / c  =', F6.3 )
C
C---- find blade angle at r/R = 0.75
      CALL SPLINE(BETA,W1,T,II)
      T75 = TINVRT(0.75)
      B75 = SEVAL(T75,BETA,W1,T,II)
C
      WRITE(*,*)
      WRITE(*,*) 'Calculating points ...'
      DO 10 L=1, NRJ
C
        ADV = RJ(L)/PI
C
C------ set freestream speed for this advance ratio
        VEL = ADV*VSO*AMACH
C
        KLOW(L) = 0
        KUPP(L) = 0
C
        IF(VEL/VSO .GT. 0.95) THEN
         WRITE(*,1350) RJ(L), VEL/VSO
 1350    FORMAT(/ ' J =',F6.3,
     &            '  yields excessive freestream Mach =', F7.3
     &          / ' skipping to next J')
         GO TO 10
        ENDIF
C
C------ set approximate zero-power beta for this advance ratio
C---HHY had to set A0 to 0.0 as A0 is now section property
        A0 = 0.0
        BZERO = ATAN2( ADV , 0.75 ) + A0 + 0.09
        DB = BZERO - B75
        DO I=1, II
          BETA(I) = BETSAV(I) + DB
        ENDDO
C
        write(*,*) 'initial Btip = ', beta(ii) * 180.0/pi

        WRITE(*,*)
C
C------ calculate points for increasing Cp's
        DO 160 K=1, NCP
C
C-------- calculate specified-power case
          PSPEC = CP(K) * 4.0/(PI*ADV)**3
          PSPEC = PSPEC * (RHO*VEL**3*RAD**2)
C
          IF(K.EQ.1) THEN
            CALL APER(3,1,.TRUE.)
          ELSE
            CALL APER(3,1,.FALSE.)
          ENDIF
C
          IF(CONV .AND. TTOT/PTOT .GT. 0.0) THEN
C
C--------- point converged and efficiency is positive, 
C          so if lower limit hasn't been set, set it now
           IF(KLOW(L).EQ.0) KLOW(L) = K
C
          ELSE
C
C--------- point bombed or efficiency is negative, 
C          so if lower limit has been set, go to next J
           IF(KLOW(L).GT.0) GO TO 161
C
          ENDIF
C
C-------- everything converged OK, so save calculated parameters
          EF(L,K) = TTOT/PTOT
          BE(L,K) = BETA(II)
          KUPP(L) = K
          WRITE(*,1030) L, K, RJ(L), CP(K), EF(L,K), BE(L,K)*180.0/PI
 1030     FORMAT(1X,2I4,'  J =', F7.4,'  Cp =', F7.4,
     &              '      eff =', F7.4,'  Btip =', F6.2    )
C
  160   CONTINUE
  161   CONTINUE
C
   10 CONTINUE
C
C---- restore beta 
      DO I=1, II
        BETA(I) = BETSAV(I)
      ENDDO
C
C---- set reference beta at r/R = 0.75, and change to degrees
ccc   DB = B75 - BETSAV(II)
C
C---- reference beta at r/R = 1.0
      DB = 0.0
C
      DO L=1, NRJ
        DO K=1, NCP
          BE(L,K) = (BE(L,K) + DB) * 180.0/PI
        ENDDO
      ENDDO
C
C---- write dump file
      LU = LUTEMP
      OPEN(LU,FILE=FNAME,STATUS='UNKNOWN',FORM='UNFORMATTED')
C
      RJSAV = ADVSAV*PI
      WRITE(LU) NAME
      WRITE(LU) RE6/RJSAV , AMACH/RJSAV
      WRITE(LU) NRJ, NCP
      WRITE(LU) (KLOW(L),L=1,NRJ)
      WRITE(LU) (KUPP(L),L=1,NRJ)
      WRITE(LU) (RJ(L),L=1,NRJ)
      WRITE(LU) (CP(K),K=1,NCP)
C
      DO K=1, NCP
        WRITE(LU) (EF(L,K),L=1,NRJ)
      ENDDO
C
      DO K=1, NCP
        WRITE(LU) (BE(L,K),L=1,NRJ)
      ENDDO
C
      CLOSE(LU)
C
      RETURN
      END ! JMAP

