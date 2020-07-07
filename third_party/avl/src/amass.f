C***********************************************************************
C    Module:  amass.f
C 
C    Copyright (C) 2002 Mark Drela, Harold Youngren
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
 
      SUBROUTINE MASINI
      INCLUDE 'AVL.INC'
C---------------------------------------------------
C     Initializes default mass, inertia tensor
C---------------------------------------------------
      RMASS0 = 1.0
C
      DO K = 1, 3
        RINER0(K,1) = 0.
        RINER0(K,2) = 0.
        RINER0(K,3) = 0.
        RINER0(K,K) = 1.0
C
        AMASS(K,1) = 0.
        AMASS(K,2) = 0.
        AMASS(K,3) = 0.
C
        AINER(K,1) = 0.
        AINER(K,2) = 0.
        AINER(K,3) = 0.
      ENDDO
C
      XYZMASS0(1) = 0.
      XYZMASS0(2) = 0.
      XYZMASS0(3) = 0.
C
      LMASS = .FALSE.
C
      RETURN
      END ! MASINI



      SUBROUTINE MASGET(LU,FNAME,ERROR)
C--------------------------------------------------
C     Reads mass distributions file,
C     computes default mass, inertia tensor
C--------------------------------------------------
      INCLUDE 'AVL.INC'
      CHARACTER*(*) FNAME
C
      REAL mass, mi
      REAL Ixx , Iyy , Izz , Ixy , Ixz , Iyz
      REAL Ixxi, Iyyi, Izzi, Ixyi, Ixzi, Iyzi
C
      CHARACTER*80 CASE
      CHARACTER*256 LINE, LINEU
C
      CHARACTER*32 UNCHGEE, UNCHRHO, UNCH
C
      LOGICAL ERROR
      REAL RINP(10), FAC(10), ADD(10)
C
 1000 FORMAT(A)
C
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=90)
C
      sum_m   = 0.
      sum_mx  = 0.
      sum_my  = 0.
      sum_mz  = 0.
      sum_mxx = 0.
      sum_myy = 0.
      sum_mzz = 0.
      sum_mxy = 0.
      sum_mxz = 0.
      sum_myz = 0.
      sum_mzz = 0.
      sum_ixx = 0.
      sum_iyy = 0.
      sum_izz = 0.
      sum_ixy = 0.
      sum_iyz = 0.
      sum_ixz = 0.
C
      xcg = 0.
      ycg = 0.
      zcg = 0.
C
C---- default multipliers and adders
      DO K = 1, 10
        FAC(K) = 1.0
        ADD(K) = 0.0
      ENDDO
C
      UNITL = 1.
      UNITM = 1.
      UNITT = 1.
      UNCHL = 'Lunit'
      UNCHM = 'Munit'
      UNCHT = 'Tunit'
      NUL = 5
      NUM = 5
      NUT = 5
C
      GEE0 = 1.
      RHO0 = 1.
      UNCHGEE = 'Lunit/Tunit^2'
      UNCHRHO = 'Munit/Lunit^3'
      NUGEE = 13
      NURHO = 13
C
      ILINE = 0

C---- search all lines of file for data values
C=============================================
 10   CONTINUE
        READ(LU,1000,END=50) LINE
        ILINE = ILINE + 1
C
        IF(INDEX('#!',LINE(1:1)).NE.0) GO TO 10
C
        IF(INDEX('*',LINE(1:1)).NE.0) THEN
C------- read multiplier line
         KINP = 10
         CALL GETFLT(LINE(2:80),RINP,KINP,ERROR)
         IF(ERROR) GO TO 40
C
         DO K = 1, KINP
           FAC(K) = RINP(K)
         ENDDO
         GO TO 10
        ENDIF
C
        IF(INDEX('+',LINE(1:1)).NE.0) THEN
C------- read adder line
         KINP = 10
         CALL GETFLT(LINE(2:80),RINP,KINP,ERROR)
         IF(ERROR) GO TO 40
C
         DO K = 1, KINP
           ADD(K) = RINP(K)
         ENDDO
         GO TO 10
        ENDIF
C
C
C------ don't look for special parameters if there's no "=" character
        KEQ = INDEX(LINE,'=')
        IF(KEQ.LE.1) GO TO 20
C
C------------------------------------------
C------ look for parameter keywords in front of "=" character
C
        K = INDEX(LINE(1:KEQ-1),'Lunit')
        IF(K .NE. 0) THEN
         LINEU = LINE(KEQ+1:256)
         CALL STRIP(LINEU,NLN)
         KB = INDEX(LINEU,' ')
         READ(LINEU(1:KB),*,ERR=40) UNITL
         UNCHL = LINEU(KB+1:256)
         CALL STRIP(UNCHL,NUL)
         NUL = MAX(NUL,1)
         GO TO 10
        ENDIF
C
        K = INDEX(LINE(1:KEQ-1),'Munit')
        IF(K .NE. 0) THEN
         LINEU = LINE(KEQ+1:256)
         CALL STRIP(LINEU,NLN)
         KB = INDEX(LINEU,' ')
         READ(LINEU(1:KB),*,ERR=40) UNITM
         UNCHM = LINEU(KB+1:256)
         CALL STRIP(UNCHM,NUM)
         NUM = MAX(NUM,1)
         GO TO 10
        ENDIF
C
        K = INDEX(LINE(1:KEQ-1),'Tunit')
        IF(K .NE. 0) THEN
         LINEU = LINE(KEQ+1:256)
         CALL STRIP(LINEU,NLN)
         KB = INDEX(LINEU,' ')
         READ(LINEU(1:KB),*,ERR=40) UNITT
         UNCHT = LINEU(KB+1:256)
         CALL STRIP(UNCHT,NUT)
         NUT = MAX(NUT,1)
         GO TO 10
        ENDIF
C
C
        K = INDEX(LINE(1:KEQ-1),'g')
        IF(K .NE. 0) THEN
         LINEU = LINE(KEQ+1:256)
         CALL STRIP(LINEU,NLN)
         KB = INDEX(LINEU,' ')
         READ(LINEU(1:KB),*,ERR=40) GEE0
         UNCHGEE = LINEU(KB+1:256)
         CALL STRIP(UNCHGEE,NUGEE)
         NUGEE = MAX(NUGEE,1)
         GO TO 10
        ENDIF
C
        K = INDEX(LINE(1:KEQ-1),'rho')
        IF(K .NE. 0) THEN
         LINEU = LINE(KEQ+1:256)
         CALL STRIP(LINEU,NLN)
         KB = INDEX(LINEU,' ')
         READ(LINEU(1:KB),*,ERR=40) RHO0
         UNCHRHO = LINEU(KB+1:256)
         CALL STRIP(UNCHRHO,NURHO)
         NURHO = MAX(NURHO,1)
         GO TO 10
        ENDIF
C
C------------------------------
 20     CONTINUE
C
        DO I=1, 10
          RINP(I) = 0.
        ENDDO
C
        KINP = 10
        CALL GETFLT(LINE,RINP,KINP,ERROR)
        IF(ERROR) GO TO 40
C
        mi = FAC(1)*RINP(1) + ADD(1)
        xi = FAC(2)*RINP(2) + ADD(2)
        yi = FAC(3)*RINP(3) + ADD(3)
        zi = FAC(4)*RINP(4) + ADD(4)
        Ixxi = FAC(5)*RINP(5) + ADD(5)
        Iyyi = FAC(6)*RINP(6) + ADD(6)
        Izzi = FAC(7)*RINP(7) + ADD(7)
        Ixyi = FAC(8)*RINP(8) + ADD(8)
        Ixzi = FAC(9)*RINP(9) + ADD(9)
        Iyzi = FAC(10)*RINP(10) + ADD(10)
C
        sum_m   = sum_m   + mi
        sum_mx  = sum_mx  + mi*xi
        sum_my  = sum_my  + mi*yi
        sum_mz  = sum_mz  + mi*zi
        sum_mxx = sum_mxx + mi*xi*xi
        sum_myy = sum_myy + mi*yi*yi
        sum_mzz = sum_mzz + mi*zi*zi
        sum_mxy = sum_mxy + mi*xi*yi
        sum_mxz = sum_mxz + mi*xi*zi
        sum_myz = sum_myz + mi*yi*zi
c
        sum_ixx = sum_ixx + Ixxi
        sum_iyy = sum_iyy + Iyyi
        sum_izz = sum_izz + Izzi
        sum_ixy = sum_ixy + Ixyi
        sum_ixz = sum_ixz + Ixzi
        sum_iyz = sum_iyz + Iyzi
C
      GO TO 10
C
C-----------------------------------
 40   WRITE(*,1500) ILINE, LINE(1:80)
 1500 FORMAT(' * Bad data line',I5,'  ignored:  ',A)
      GO TO 10
C
C=============================================
C
 50   CONTINUE
      CLOSE(LU)
C
      mass = sum_m
C
      xcg = sum_mx/sum_m
      ycg = sum_my/sum_m
      zcg = sum_mz/sum_m
C
      Ixx = sum_ixx + (sum_myy + sum_mzz) - sum_m*(ycg**2 + zcg**2)
      Iyy = sum_iyy + (sum_mzz + sum_mxx) - sum_m*(zcg**2 + xcg**2)
      Izz = sum_izz + (sum_mxx + sum_myy) - sum_m*(xcg**2 + ycg**2)
      Ixy = sum_ixy +  sum_mxy            - sum_m* xcg*ycg
      Ixz = sum_ixz +  sum_mxz            - sum_m* xcg*zcg
      Iyz = sum_iyz +  sum_myz            - sum_m* ycg*zcg
C
      RMASS0 = mass * UNITM
C
      RINER0(1,1) =  Ixx * UNITM*UNITL**2
      RINER0(1,2) = -Ixy * UNITM*UNITL**2
      RINER0(1,3) = -Ixz * UNITM*UNITL**2
C
      RINER0(2,1) = -Ixy * UNITM*UNITL**2
      RINER0(2,2) =  Iyy * UNITM*UNITL**2
      RINER0(2,3) = -Iyz * UNITM*UNITL**2
C
      RINER0(3,1) = -Ixz * UNITM*UNITL**2
      RINER0(3,2) = -Iyz * UNITM*UNITL**2
      RINER0(3,3) =  Izz * UNITM*UNITL**2
C
      XYZMASS0(1) = xcg * UNITL
      XYZMASS0(2) = ycg * UNITL
      XYZMASS0(3) = zcg * UNITL
C
C----------------------------------------
C---- set derived unit values and names
      CALL UNITSET
C
C---- set parameter unit names
      CALL PARNSET
C
      ERROR = .FALSE.
      LMASS = .TRUE.
      RETURN
C
 90   CONTINUE
      CALL STRIP(FNAME,NFN)
      WRITE(*,9000) FNAME(1:NFN)
 9000 FORMAT(/' Mass file  ',A,'  open error')
      ERROR = .TRUE.
      LMASS = .FALSE.
      RETURN
      END ! MASGET



      SUBROUTINE MASPUT(IR1,IR2)
      INCLUDE 'AVL.INC'
C-------------------------------------------
C     Stores default mass, inertias 
C     in run case parameter array
C-------------------------------------------
C
      DO IR = IR1, IR2
        PARVAL(IPMASS,IR) = RMASS0
        PARVAL(IPIXX,IR) = RINER0(1,1)
        PARVAL(IPIYY,IR) = RINER0(2,2)
        PARVAL(IPIZZ,IR) = RINER0(3,3)
        PARVAL(IPIXY,IR) = RINER0(1,2)
        PARVAL(IPIYZ,IR) = RINER0(2,3)
        PARVAL(IPIZX,IR) = RINER0(3,1)

        PARVAL(IPGEE,IR) = GEE0
        PARVAL(IPRHO,IR) = RHO0
C
        PARVAL(IPXCG,IR) = XYZMASS0(1)/UNITL
        PARVAL(IPYCG,IR) = XYZMASS0(2)/UNITL
        PARVAL(IPZCG,IR) = XYZMASS0(3)/UNITL
      ENDDO
C
      RETURN
      END ! MASPUT



      SUBROUTINE MASSHO(LU)
      INCLUDE 'AVL.INC'
C
      WRITE(LU,*)
      IF(UNCHM(1:NUM).NE.'Munit')
     &WRITE(LU,1250) 'Mass        = ', RMASS0/UNITM, 'Munit'
      WRITE(LU,1250) 'Mass        = ', RMASS0, UNCHM(1:NUM)
C
      WRITE(LU,*)
      WRITE(LU,1260) 'Ref. x,y,z  = ',(XYZREF0(K)       , K=1,3),'Lunit'
      IF(UNCHL(1:NUL).NE.'Lunit')
     &WRITE(LU,1260) 'C.G. x,y,z  = ',(XYZMASS0(K)/UNITL, K=1,3),'Lunit'
      WRITE(LU,1260) 'C.G. x,y,z  = ',(XYZMASS0(K), K=1,3), UNCHL(1:NUL)
C
      WRITE(LU,*)
      WRITE(LU,1271) 'Ixx -Ixy -Ixz   | ',
     &            RINER0(1,1),RINER0(1,2),RINER0(1,3),'|'
      WRITE(LU,1272) '     Iyy -Iyz = | ',
     &                        RINER0(2,2),RINER0(2,3),'|', UNCHI(1:NUI)
      WRITE(LU,1273) '          Izz   | ',
     &                                    RINER0(3,3),'|'
C
 1250 FORMAT(1X, A,  G12.4,2X,A)
 1260 FORMAT(1X, A, 3G12.4,2X,A)
 1271 FORMAT(1X, A,      3G12.4, 2X, A, 2X, A)
 1272 FORMAT(1X, A, 12X, 2G12.4, 2X, A, 2X, A)
 1273 FORMAT(1X, A, 24X,  G12.4, 2X, A, 2X, A)
C
      RETURN
      END ! MASSHO


      SUBROUTINE APPSHO(LU,RHO)
      INCLUDE 'AVL.INC'
C
      WRITE(LU,*) 'Apparent mass, inertia'
      WRITE(LU,*)
      WRITE(LU,1271) 'mxx  mxy  mxz   | ',
     &  AMASS(1,1)*RHO,AMASS(1,2)*RHO,AMASS(1,3)*RHO,'|'
      WRITE(LU,1272) '     myy  myz = | ',
     &                 AMASS(2,2)*RHO,AMASS(2,3)*RHO,'|', UNCHM(1:NUM)
      WRITE(LU,1273) '          mzz   | ',
     &                                AMASS(3,3)*RHO,'|'
      WRITE(LU,*)
      WRITE(LU,1271) 'Ixx -Ixy -Ixz   | ',
     &  AINER(1,1)*RHO,AINER(1,2)*RHO,AINER(1,3)*RHO,'|'
      WRITE(LU,1272) '     Iyy -Iyz = | ',
     &                 AINER(2,2)*RHO,AINER(2,3)*RHO,'|', UNCHI(1:NUI)
      WRITE(LU,1273) '          Izz   | ',
     &                                AINER(3,3)*RHO,'|'
C
 1271 FORMAT(1X, A,      3G12.4, 2X, A, 2X, A)
 1272 FORMAT(1X, A, 12X, 2G12.4, 2X, A, 2X, A)
 1273 FORMAT(1X, A, 24X,  G12.4, 2X, A, 2X, A)
C
      RETURN
      END ! APPSHO



      SUBROUTINE APPGET
C----------------------------------------------------------------
C     Calculates apparent mass and inertia for unit air density
C----------------------------------------------------------------
      INCLUDE 'AVL.INC'
      REAL UC(3), US(3), UN(3), RM(3), RXUN(3)
C
      DO K = 1, 3
        DO L = 1, 3
          AMASS(K,L) = 0.
          AINER(K,L) = 0.
        ENDDO
      ENDDO
C
      DO 100 J = 1, NSTRIP
        CR = CHORD(J)
        SR = CHORD(J)*WSTRIP(J)
C
C------ strip normal unit vector UN
        UN(1) = 0.0
        UN(2) = ENSY(J)
        UN(3) = ENSZ(J)
C
C------ spanwise unit vector US (along midchord)
        US(1) = RLE2(1,J) - RLE1(1,J) + 0.5*(CHORD2(J) - CHORD1(J))
        US(2) = RLE2(2,J) - RLE1(2,J)
        US(3) = RLE2(3,J) - RLE1(3,J)
        UMAG = SQRT(US(1)*US(1) + US(2)*US(2) + US(3)*US(3))
        IF(UMAG.GT.0.0) THEN
         US(1) = US(1)/UMAG
         US(2) = US(2)/UMAG
         US(3) = US(3)/UMAG
        ENDIF
C
C------ perpendicular-chord unit vector UC
        CALL CROSS(US,UN,UC)
C
C------ use the strip 1/2 chord location for moment arm
        RM(1) = RLE(1,J) + 0.5*CR
        RM(2) = RLE(2,J)
        RM(3) = RLE(3,J)
C
        CALL CROSS(RM,UN,RXUN)
C
C------ perpendicular chord
        CPERP = CR * (US(2)*UN(3) - US(3)*UN(2))
C
C------ apparent mass and spanwise-axis apparent inertia of strip
        APPM = SR * 0.25*PI*CPERP          
        APPI = SR * 0.25*PI*CPERP**3 / 64.0
C
C------ add apparent mass contribution
        DO K = 1, 3
          DO L = 1, 3
            AMASS(K,L) = AMASS(K,L) + APPM*  UN(K)*  UN(L) * UNITL**3
            AINER(K,L) = AINER(K,L) + APPM*RXUN(K)*RXUN(L) * UNITL**5
     &                              + APPI*  US(K)*  US(L) * UNITL**5
          ENDDO
        ENDDO
C
  100 CONTINUE
C
      RETURN
      END ! APPGET



      SUBROUTINE UNITSET
      INCLUDE 'AVL.INC'

C----------------------------------------
C---- set force unit
      UNITF = UNITM*UNITL/UNITT**2
      IF(UNCHM(1:1).NE.' ' .AND.
     &   UNCHL(1:1).NE.' ' .AND.
     &   UNCHT(1:1).NE.' '      ) THEN
       UNCHF = UNCHM(1:NUM)//'-'//UNCHL(1:NUL)//'/'//UNCHT(1:NUT)//'^2'
       CALL STRIP(UNCHF,NUF)
       IF(UNCHF(1:NUF).EQ.'slug-ft/s^2') UNCHF = 'lb'
       IF(UNCHF(1:NUF).EQ.'kg-m/s^2'   ) UNCHF = 'N'
       IF(UNCHF(1:NUF).EQ.'g-cm/s^2'   ) UNCHF = 'dyn'
       CALL STRIP(UNCHF,NUF)
      ENDIF
C
C---- set area unit
      UNITS = UNITL**2
      IF(UNCHL(1:1).NE.' ') THEN
       UNCHS = UNCHL(1:NUL)//'^2'
       CALL STRIP(UNCHS,NUS)
      ENDIF
C 
C---- set velocity unit
      UNITV = UNITL/UNITT
      IF(UNCHL(1:1).NE.' ' .AND.
     &   UNCHT(1:1).NE.' '      ) THEN
       UNCHV = UNCHL(1:NUL)//'/'//UNCHT(1:NUT)
       CALL STRIP(UNCHV,NUV)
      ENDIF
C 
C---- set acceleration unit name
      UNITA = UNITL/UNITT**2
      IF(UNCHL(1:1).NE.' ' .AND.
     &   UNCHT(1:1).NE.' '      ) THEN
       UNCHA = UNCHL(1:NUL)//'/'//UNCHT(1:NUT)//'^2'
       CALL STRIP(UNCHA,NUA)
      ENDIF
C 
C---- set inertia unit name
      UNITI = UNITM*UNITL**2
      IF(UNCHM(1:1).NE.' ' .AND.
     &   UNCHL(1:1).NE.' '      ) THEN
       UNCHI = UNCHM(1:NUM)//'-'//UNCHL(1:NUL)//'^2'
       CALL STRIP(UNCHI,NUI)
      ENDIF
C
C---- set density unit name
      UNITD = UNITM/UNITL**3
      IF(UNCHM(1:1).NE.' ' .AND.
     &   UNCHL(1:1).NE.' '      ) THEN
       UNCHD = UNCHM(1:NUM)//'/'//UNCHL(1:NUL)//'^3'
       CALL STRIP(UNCHD,NUD)
      ENDIF
C
      RETURN
      END ! UNITSET

