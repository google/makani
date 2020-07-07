C***********************************************************************
C    Module:  xinte.f
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

      SUBROUTINE INTE1
      INCLUDE 'XROTOR.INC'
C-----------------------------------------------
C     Interpolates geometry to specified radii
C-----------------------------------------------
      CHARACTER*80 FLIST
      LOGICAL LSAV
C
      GREEK = .FALSE.
C
      CALL SPLINE(CH,  T1,T,II)
      CALL SPLINE(BETA,T2,T,II)
      CALL SPLINE(CL,  T3,T,II)
C
      CALL SPLINE(CH,  T4,XI(1),II)
      CALL SPLINE(BETA,T5,XI(1),II)
      CALL SPLINE(CL,  T6,XI(1),II)
C
      LU = LUTEMP
      CALL ASKS('Enter station list filename^',FLIST)
      OPEN(LU,FILE=FLIST,STATUS='OLD')
      DO 40 IR=1, IX
        READ(LU,*,END=50) W0(IR), W9(IR)
C
        IF(W0(IR).GE.XI0) THEN
         XT = TINVRT(W0(IR))
         W1(IR) = SEVAL(XT,CH,  T1,T,II)
         W2(IR) = SEVAL(XT,BETA,T2,T,II)
         W3(IR) = SEVAL(XT,CL,  T3,T,II)
        ELSE
         W1(IR) = SEVAL(W0(IR),CH,  T4,XI(1),II)
         W2(IR) = SEVAL(W0(IR),BETA,T5,XI(1),II)
         W3(IR) = SEVAL(W0(IR),CL,  T6,XI(1),II)
        ENDIF
C
        W4(IR) = 0.5*RHO*VEL**2 * (1.0 + (W0(IR)/ADV)**2)
C
   40 CONTINUE
C
   50 NR = IR-1
      CLOSE(LU)
C
      CALL ASKL('Save to disk file ?^',LSAV)
      IF(LSAV) CALL OPFILE(LUSAVE,SAVFIL)
      WRITE(LUWRIT,1100)
ccc   WRITE(LUWRIT,1200)
      WRITE(LUWRIT,1210)
      IF(LSAV) WRITE(LUSAVE,1100)
ccc   IF(LSAV) WRITE(LUSAVE,1200)
      IF(LSAV) WRITE(LUSAVE,1210)
      DO 90 IR=1, NR
C
C------ set chord fraction in back of twist axis to 3/4 chord
        CBACK = (0.75-XPITCH)*W1(IR)
C
C------ set effective radius at current location
        DX = (CBACK*COS(W2(IR)))**2 / W0(IR)
        XMOD = W0(IR) + DX
        XMOD = AMIN1(0.99999,XMOD)
C
C------ set proper blade angle (from zero-lift line) at effective radius
C
C---HHY had to set A0 to 0.0 as A0 is now section property
        A0  = 0.0 
        IF(W0(IR).GE.XI0) THEN
         XT = TINVRT(XMOD)
         BETMOD = SEVAL(XT,BETA,T2,T,II) - A0
        ELSE
         BETMOD = SEVAL(W0(IR),BETA,T5,XI(1),II) - A0
        ENDIF
C
C------ set extended chord to permit linear interpolation between stations
        CMOD = W1(IR)
        IF(IR .GT. 1) THEN
         XMID = 0.5*(W0(IR-1) + W0(IR))
         IF(XMID.GE.XI0) THEN
          XT = TINVRT(XMID)
          CMID = SEVAL(XT,CH,T1,T,II)
         ELSE
          CMID = SEVAL(XMID,CH,T4,XI(1),II)
         ENDIF
         DCDX = (W1(IR-1) - W1(IR)) / (W0(IR-1) - W0(IR))
         CEXT = CMID + DCDX*(W0(IR)-XMID)
         CMOD = AMAX1( CMOD , CEXT )
        ENDIF
        IF(IR .LT. NR) THEN
         XMID = 0.5*(W0(IR+1) + W0(IR))
         IF(XMID.GE.XI0) THEN
          XT = TINVRT(XMID)
          CMID = SEVAL(XT,CH,T1,T,II)
         ELSE
          CMID = SEVAL(XMID,CH,T4,XI(1),II)
         ENDIF
         DCDX = (W1(IR+1) - W1(IR)) / (W0(IR+1) - W0(IR))
         CEXT = CMID + DCDX*(W0(IR)-XMID)
         CMOD = AMAX1( CMOD , CEXT )
        ENDIF
C
        WRITE(LUWRIT,1300) IR,
     &        W0(IR),
     &        W1(IR),
     &        W0(IR)*RAD        * 100.0/2.54,
     &        W1(IR)*RAD        * 100.0/2.54,
     &        BETMOD*180.0/PI + W9(IR),
     &        W9(IR),
     &        BETMOD*180.0/PI,
     &        CMOD*RAD          * 100.0/2.54,
     &        W3(IR),
     &        W4(IR)
        IF(LSAV) WRITE(LUSAVE,1300) IR,
     &        W0(IR),
     &        W1(IR),
     &        W0(IR)*RAD        * 100.0/2.54,
     &        W1(IR)*RAD        * 100.0/2.54,
     &        BETMOD*180.0/PI + W9(IR),
     &        W9(IR),
     &        BETMOD*180.0/PI,
     &        CMOD*RAD          * 100.0/2.54,
     &        W3(IR),
     &        W4(IR)
   90 CONTINUE
      WRITE(LUWRIT,1100)
      IF(LSAV) WRITE(LUSAVE,1100)
      IF(LSAV) CLOSE(LUSAVE)
C
      RETURN
C....................................................................
C
 1100 FORMAT(
     & /' =========================================================')
 1200 FORMAT(
     & '   i  r/R    c/R    radius   chord  beta    aOL   betaOL  cmod'
     &/'                      (m)    (cm)   (deg)  (deg)  (deg)   (cm)')
 1210 FORMAT(
     & '   i  r/R    c/R    radius   chord  beta    aOL   betaOL  cmod'  
     &/'                     (in)    (in)   (deg)  (deg)  (deg)   (in)')
C         10 0.250  0.4215  20.012   8.231  64.23 -2.23   66.46   8.451
 1300 FORMAT(1X,
     &    I3,F6.3,  F8.4,   F8.3,    F8.3,  F7.2,  F7.2,   F8.2,  F8.3,
     &       F8.4, F12.1 )
C
 1220 FORMAT(
     & '   i   r/R     radius   chord     beta     aOL     xTE'  
     &/'                (in)     (in)     (deg)   (deg)    (in)')
C         10 0.2500   0.4212   1.0123   42.2314  64.235   8.4516
 1320 FORMAT(1X,
     &    I3,F7.4,   F9.4,   F9.4,    F10.4,  F8.3,   F9.4 )
      END ! INTE2



      SUBROUTINE INTE2
      INCLUDE 'XROTOR.INC'
C-----------------------------------------------
C     Interpolates geometry to specified radii
C-----------------------------------------------
      CHARACTER*80 FLIST
      LOGICAL LSAV
C
      GREEK = .FALSE.
C
      CALL SPLINE(CH,  T1,T,II)
      CALL SPLINE(BETA,T2,T,II)
      CALL SPLINE(CL,  T3,T,II)
C
      CALL SPLINE(CH,  T4,XI(1),II)
      CALL SPLINE(BETA,T5,XI(1),II)
      CALL SPLINE(CL,  T6,XI(1),II)
C
      LU = LUTEMP
      CALL ASKS('Enter station list filename^',FLIST)
      OPEN(LU,FILE=FLIST,STATUS='OLD')
      DO 40 IR=1, IX
        READ(LU,*,END=50) W0(IR), W9(IR)
C
        IF(W0(IR).GE.XI0) THEN
         XT = TINVRT(W0(IR))
         W1(IR) = SEVAL(XT,CH,  T1,T,II)
         W2(IR) = SEVAL(XT,BETA,T2,T,II)
         W3(IR) = SEVAL(XT,CL,  T3,T,II)
        ELSE
         W1(IR) = SEVAL(W0(IR),CH,  T4,XI(1),II)
         W2(IR) = SEVAL(W0(IR),BETA,T5,XI(1),II)
         W3(IR) = SEVAL(W0(IR),CL,  T6,XI(1),II)
        ENDIF
C
        W4(IR) = 0.5*RHO*VEL**2 * (1.0 + (W0(IR)/ADV)**2)
C
   40 CONTINUE
C
   50 NR = IR-1
      CLOSE(LU)
C
      CALL ASKL('Save to disk file ?^',LSAV)
      IF(LSAV) CALL OPFILE(LUSAVE,SAVFIL)
C
      WRITE(LUWRIT,1100) NAME
      WRITE(LUWRIT,1220)
C
      IF(LSAV) THEN
       WRITE(LUSAVE,1100) NAME
       WRITE(LUSAVE,1220)
      ENDIF
C
      XOCTE = 0.70
      DO 90 IR=1, NR
        WRITE(LUWRIT,1320) IR,
     &        W0(IR),
     &        W0(IR)*RAD        * 100.0/2.54,
     &        W1(IR)*RAD*100.0         /2.54,
     &        W2(IR)*180.0/PI + W9(IR),
     &        W9(IR),
     &        XOCTE*W1(IR)*RAD*100.0   /2.54
        IF(LSAV) WRITE(LUSAVE,1320) IR,
     &        W0(IR),
     &        W0(IR)*RAD        * 100.0/2.54,
     &        W1(IR)*RAD*100.0         /2.54,
     &        W2(IR)*180.0/PI + W9(IR),
     &        W9(IR),
     &        XOCTE*W1(IR)*RAD*100.0   /2.54
   90 CONTINUE
      IF(LSAV) CLOSE(LUSAVE)
C
      RETURN
C....................................................................
C
 1100 FORMAT(/1X, A /)
 1220 FORMAT(
     & '   i   r/R     radius   chord     beta     aOL     xTE'  
     &/'                (in)     (in)     (deg)   (deg)    (in)')
C         10 0.2500   0.4210   1.0120   42.2310  64.230   8.4510
 1320 FORMAT(1X,
     &    I3,F7.4,   F9.4,   F9.4,    F10.4,  F8.3,   F9.4 )
      END ! INTE2

