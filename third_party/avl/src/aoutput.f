C***********************************************************************
C    Module:  aoutput.f
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

      SUBROUTINE OUTTOT(LUN)
C
C...PURPOSE  To print out results of the vortex lattice calculation
C            for the input configuration.  
C
C...INPUT    Configuration data for case in labeled commons
C          
C...OUTPUT   Printed output on logical unit LUN
C
      INCLUDE 'AVL.INC'
      CHARACTER*50 SATYPE
C
 1000 FORMAT (A)
C
      IF (LUN.EQ.0) RETURN
C
C
      CALL GETSA(LNASA_SA,SATYPE,DIR)
C
      CA = COS(ALFA)
      SA = SIN(ALFA)
C
C---- set normalized rates in Stability or Body axes
      RX_S = (WROT(1)*CA + WROT(3)*SA) * BREF/2.0
      RY_S =  WROT(2)                  * CREF/2.0
      RZ_S = (WROT(3)*CA - WROT(1)*SA) * BREF/2.0
      RX_B =  WROT(1) * BREF/2.0
      RY_B =  WROT(2) * CREF/2.0
      RZ_B =  WROT(3) * BREF/2.0
C
C---- set body forces in Geometric axes
      CXTOT = CDTOT*CA - CLTOT*SA
      CZTOT = CDTOT*SA + CLTOT*CA
C
C---- set moments in stability axes
      CRSAX = CRTOT*CA + CNTOT*SA
      CMSAX = CMTOT              
      CNSAX = CNTOT*CA - CRTOT*SA  
CCC   CNSAX = CNTOT*CA - CRTOT*CA        !!! Bug   MD  02 Apr 04
C
C---- dump it
      WRITE(LUN,200)
      WRITE(LUN,201) 
      WRITE(LUN,202) TITLE(1:60),NSURF,NSTRIP,NVOR
      IF(IYSYM.GT.0) WRITE (*,2034) YSYM
      IF(IYSYM.LT.0) WRITE (*,2035) YSYM
      IF(IZSYM.GT.0) WRITE (*,2036) ZSYM
      IF(IZSYM.LT.0) WRITE (*,2037) ZSYM

      WRITE(LUN,204) SREF,CREF,BREF, 
     &               XYZREF(1), XYZREF(2), XYZREF(3)
C
      WRITE(LUN,205) SATYPE

      WRITE(LUN,218) RTITLE(IRUN)
      WRITE(LUN,220) ALFA/DTR, DIR*RX_B, DIR*RX_S, 
     &               BETA/DTR,     RY_B,
     &               AMACH   , DIR*RZ_B, DIR*RZ_S
C
      CDITOT = CDTOT - CDVTOT
      WRITE (LUN,221) DIR*CXTOT ,  DIR*CRTOT,  DIR*CRSAX,
     &                    CYTOT ,      CMTOT,
     &                DIR*CZTOT ,  DIR*CNTOT,  DIR*CNSAX,
     &                CLTOT ,
     &                CDTOT ,
     &                CDVTOT,  CDITOT,
     &                CLFF  ,  CDFF  ,
     &                CYFF  ,  SPANEF
C
      WRITE(LUN,*)
      DO K = 1, NCONTROL
        WRITE(LUN,231) DNAME(K), DELCON(K)
      ENDDO

      WRITE(LUN,*)
      DO K = 1, NDESIGN
        WRITE(LUN,231) GNAME(K), DELDES(K)
      ENDDO
C
      WRITE(LUN,200)
 200  FORMAT(1X,
     &'---------------------------------------------------------------')
 201  FORMAT(' Vortex Lattice Output -- Total Forces')
 202  FORMAT(/' Configuration: ',A
     &       /5X,'# Surfaces =',I4
     &       /5X,'# Strips   =',I4
     &       /5X,'# Vortices =',I4)
C
 2034 FORMAT(' Y Symmetry: Wall plane   at Ysym =',F10.4)
 2035 FORMAT(' Y Symmetry: Free surface at Ysym =',F10.4)
 2036 FORMAT(' Z Symmetry: Ground plane at Zsym =',F10.4)
 2037 FORMAT(' Z Symmetry: Free surface at Zsym =',F10.4)
C
 204  FORMAT(/2X, 'Sref =',G12.5,3X,'Cref =',G12.5,3X,'Bref =',G12.5
     &       /2X, 'Xref =',G12.5,3X,'Yref =',G12.5,3X,'Zref =',G12.5 )

 205  FORMAT(/1X, A)
 218  FORMAT(/' Run case: ', A)
 220  FORMAT(
     & /2X,'Alpha =',F10.5,5X,'pb/2V =',F10.5,5X,'p''b/2V =',F10.5
     & /2X,'Beta  =',F10.5,5X,'qc/2V =',F10.5
     & /2X,'Mach  =',F10.3,5X,'rb/2V =',F10.5,5X,'r''b/2V =',F10.5)
C
 221  FORMAT (
     & /2X,'CXtot =',F10.5,5X,'Cltot =',F10.5,5X,'Cl''tot =',F10.5
     & /2X,'CYtot =',F10.5,5X,'Cmtot =',F10.5 
     & /2X,'CZtot =',F10.5,5X,'Cntot =',F10.5,5X,'Cn''tot =',F10.5
     &//2X,'CLtot =',F10.5
     & /2X,'CDtot =',F10.5
     & /2X,'CDvis =',F10.5,5X,'CDind =',F10.5
     & /2X,'CLff  =',F10.5,5X,'CDff  =',F10.5,4X,'| Trefftz'
     & /2X,'CYff  =',F10.5,5X,'    e =',F10.4,4X,'| Plane  ' )
C
 231  FORMAT(3X,A,'=',F10.5)
C
 240  FORMAT (/)
C
      RETURN
      END ! OUTTOT


      SUBROUTINE OUTSURF(LUN)
C
C...PURPOSE  To print out surface forces from the vortex lattice calculation
C
C...INPUT    Configuration data for case in labeled commons
C          
C...OUTPUT   Printed output for each surface on logical unit LUN
C
      INCLUDE 'AVL.INC'
      CHARACTER*50 SATYPE
C
 1000 FORMAT (A)
C
      IF (LUN.EQ.0) RETURN
C
      CALL GETSA(LNASA_SA,SATYPE,DIR)
C
C...Print out the results
C
C...Force components from each surface
      WRITE(LUN,200)
 200  FORMAT(1X,
     &'---------------------------------------------------------------')
      WRITE (LUN,210) SATYPE,
     &                SREF,CREF,BREF, 
     &                XYZREF(1), XYZREF(2), XYZREF(3)
      DO N = 1, NSURF
        CALL STRIP(STITLE(N),NT)
        WRITE (LUN,211) N,SSURF(N),CLSURF(N),CDSURF(N),CMSURF(N),
     &                  CYSURF(N),DIR*CNSURF(N),DIR*CRSURF(N),
     &                  CDSURF(N)-CDVSURF(N),CDVSURF(N),
     &                  STITLE(N)(1:NT)
      END DO
cc      WRITE(LUN,212)
C
C--- Surface forces normalized by local reference quantities
      WRITE (LUN,220) 
      DO N = 1, NSURF
        CALL STRIP(STITLE(N),NT)
        WRITE (LUN,221) N,SSURF(N),CAVESURF(N),
     &                  CL_SRF(N),CD_SRF(N),
     &                  CDVSURF(N)*SREF/SSURF(N),CMLE_SRF(N),
     &                  STITLE(N)(1:NT)
      END DO
      WRITE(LUN,200)
C
  210 FORMAT ( ' Surface Forces (referred to Sref,Cref,Bref',
     &                                   ' about Xref,Yref,Zref)',
     &        /' ',A //
     &      5X,'Sref =',G12.4,   3X,'Cref =',F10.4,3X,'Bref =',F10.4/
     &      5X,'Xref =',2X,F10.4,3X,'Yref =',F10.4,3X,'Zref =',F10.4//
     &     ' n',6X,'Area',6X,'CL',6X,'CD',6X,'Cm',
     &                    6X,'CY',6X,'Cn',6X,'Cl',5X,'CDi',5X,'CDv')
  211 FORMAT (I2,1X,F9.3,8F8.4,3X,A)
  212 FORMAT (/)
C
  220 FORMAT (/' Surface Forces (referred to Ssurf, Cave ',
     &         'about root LE on hinge axis)'//
     &        2X,' n',5X,'Ssurf',6X,'Cave',
     &        7X,'cl',7X,'cd',6X,'cdv',4x,'cm_LE')
  221 FORMAT (2X,I2,F10.3,F10.3,4(1X,F8.4),2X,A)
C
      RETURN
      END ! OUTSURF


      SUBROUTINE OUTBODY(LUN)
C
C...PURPOSE  To print out body forces from source/doublet calculation
C
C...INPUT    Configuration data for case in labeled commons
C          
C...OUTPUT   Printed output for each body on logical unit LUN
C
      INCLUDE 'AVL.INC'
      CHARACTER*50 SATYPE
C
 1000 FORMAT (A)
C
      IF (LUN.EQ.0) RETURN
C
      CALL GETSA(LNASA_SA,SATYPE,DIR)
C
C...Print out the results
C
C...Force components from each body 
      WRITE(LUN,200)
 200  FORMAT(1X,
     &'---------------------------------------------------------------')
      WRITE (LUN,210) SATYPE,
     &                SREF,CREF,BREF, 
     &                XYZREF(1), XYZREF(2), XYZREF(3)
      DO IB = 1, NBODY
        CALL STRIP(BTITLE(IB),NT)
        ELBD = ELBDY(IB)
        SBDY = SRFBDY(IB)
        VBDY = VOLBDY(IB)
        WRITE (LUN,211) IB,ELBD,SBDY,VBDY,
     &                  CLBDY(IB),CDBDY(IB),CMBDY(IB),
     &                  CYBDY(IB),DIR*CNBDY(IB),DIR*CRBDY(IB),  
     &                  BTITLE(IB)(1:NT)
      END DO
cc      WRITE(LUN,212)
      WRITE(LUN,200)
C
  210 FORMAT ( ' Body Forces (referred to Sref,Cref,Bref',
     &                            ' about Xref,Yref,Zref)',
     &        /' ',A //
     &      5X,'Sref =',G12.4,   3X,'Cref =',F10.4,3X,'Bref =',F10.4/
     &      5X,'Xref =',2X,F10.4,3X,'Yref =',F10.4,3X,'Zref =',F10.4//
     &     'Ibdy',4X,'Length',5X,'Asurf',7X,'Vol',
     &      6X,'CL',6X,'CD',6X,'Cm',
     &      6X,'CY',6X,'Cn',6X,'Cl')
  211 FORMAT (I4,3(1X,F9.3),6F8.4,3X,A)
  212 FORMAT (/)
C
      RETURN
      END ! OUTBODY



      SUBROUTINE OUTSTRP(LUN)
C
C...PURPOSE  To print out results of the vortex lattice calculation
C            for the input configuration strip and surface forces.  
C
C...INPUT    Configuration data for case in labeled commons
C          
C...OUTPUT   Printed output on logical unit LUN
C
      INCLUDE 'AVL.INC'
      CHARACTER*50 SATYPE
C
 1000 FORMAT (A)
C
      IF (LUN.EQ.0) RETURN
C
      CALL GETSA(LNASA_SA,SATYPE,DIR)
C
C...Print out the results -> Forces by surface and strip
      WRITE(LUN,200)
      WRITE(LUN,210) 
      WRITE(LUN,212) SATYPE
      DO N = 1, NSURF
        NS = NJ(N)
        NV = NK(N)
        J1 = JFRST(N)
C
        WRITE (LUN,211) N,STITLE(N),NV,NS,J1,SSURF(N),CAVESURF(N)
        CDISURF = CDSURF(N)-CDVSURF(N)
        WRITE (LUN,213) CLSURF(N),DIR*CRSURF(N),
     &                  CYSURF(N),    CMSURF(N),
     &                  CDSURF(N),DIR*CNSURF(N),
     &                  CDISURF,CDVSURF(N)
        WRITE (LUN,214) CL_SRF(N),CD_SRF(N)
        WRITE (LUN,216) 
        DO JJ = 1, NS
          J = J1 + JJ-1
          ASTRP = WSTRIP(J)*CHORD(J)
          XCP = 999.
cc        IF(CLSTRP(J).NE.0.)  XCP = 0.25 - CMC4(J)/CLSTRP(J)       !!! BUG  13 Jan 12   MD
          IF(CL_LSTRP(J).NE.0.)  XCP = 0.25 - CMC4(J)/CL_LSTRP(J)
          IF(XCP.LT.2.0 .AND. XCP.GT.-1.0) THEN
            WRITE (LUN,217)
     &            J,RLE(2,J),CHORD(J),ASTRP,CNC(J),DWWAKE(J),
     &            CLTSTRP(J), CL_LSTRP(J),CD_LSTRP(J),CDV_LSTRP(J),
     &            CMC4(J),CMLE(J),XCP
           ELSE
            WRITE (LUN,217)
     &            J,RLE(2,J),CHORD(J),ASTRP,CNC(J),DWWAKE(J),
     &            CLTSTRP(J),CL_LSTRP(J),CD_LSTRP(J),CDV_LSTRP(J),
     &            CMC4(J),CMLE(J)
          ENDIF
        END DO
      END DO
      WRITE(LUN,200)
C
  200 FORMAT(1X,
     &'---------------------------------------------------------------')
  210 FORMAT (' Surface and Strip Forces by surface')
  211 FORMAT (/2X,'Surface #',I2,5X,A/
     &        5X,'# Chordwise =',I3,3X,'# Spanwise =',I3,
     &        5X,'First strip =',I3/
     &        5X,'Surface area =',F12.6,5X,'  Ave. chord =',F12.6)
  212 FORMAT (/'  Forces referred to Sref, Cref, Bref ',
     &        'about Xref, Yref, Zref'/
     &         ' ',A)
  213 FORMAT ( 5X,'CLsurf  =',F10.5,5X,'Clsurf  =',F10.5,
     &        /5X,'CYsurf  =',F10.5,5X,'Cmsurf  =',F10.5,
     &        /5X,'CDsurf  =',F10.5,5X,'Cnsurf  =',F10.5, 
     &        /5X,'CDisurf =',F10.5,5x,'CDvsurf =',F10.5)
  214 FORMAT (/'  Forces referred to Ssurf, Cave ',
     &         'about hinge axis thru LE'/
     &         5X,'CLsurf  =',F10.5,5X,'CDsurf  =',F10.5/
     &         5X,'Deflect =',F10.5,5X,'CmLEsurf=',F10.5)
  216 FORMAT (/' Strip Forces referred to Strip Area, Chord'/
     &        2X,'  j ',5X,'Yle',4X,'Chord',5X,'Area',
     &        5X,'c cl',6X,'ai',6X,'cl_norm',2X,'cl',7X,'cd',7X,
     &        'cdv',4x,'cm_c/4',4x,'cm_LE',2x,'C.P.x/c')
  217 FORMAT (2X,I4,11(1X,F8.4),1X,F8.3)
C
      RETURN
      END ! OUTSTRP


      SUBROUTINE OUTELE(LUN)
      INCLUDE 'AVL.INC'
      CHARACTER*50 SATYPE
C
 1000 FORMAT (A)
C
      IF (LUN.EQ.0) RETURN
C
      CALL GETSA(LNASA_SA,SATYPE,DIR)
C
C...Forces on each strip and element (long output, and slow to printout)
      WRITE(LUN,200)
      WRITE(LUN,230)
      WRITE(LUN,212) SATYPE
      DO N = 1, NSURF
        NS = NJ(N)
        NV = NK(N)
        J1 = JFRST(N)
C
        WRITE (LUN,231) N,STITLE(N),NV,NS,J1,
     &                  SSURF(N),CAVESURF(N)
C
        CDISURF = CDSURF(N)-CDVSURF(N)
        WRITE (LUN,213) CLSURF(N),DIR*CRSURF(N),
     &                  CYSURF(N),    CMSURF(N),
     &                  CDSURF(N),DIR*CNSURF(N),
     &                  CDISURF,CDVSURF(N)
        WRITE (LUN,214) CL_SRF(N),CD_SRF(N)
C
        DO JJ = 1, NS
          J = J1 + JJ-1
          I1 = IJFRST(J)
          ASTRP = WSTRIP(J)*CHORD(J)
          DIHED = -ATAN2(ENSY(J),ENSZ(J))/DTR
          WRITE (LUN,232) J,NV,I1,
     &                    RLE(1,J),CHORD(J),AINC(J)/DTR,
     &                    RLE(2,J),WSTRIP(J),ASTRP,
     &                    RLE(3,J),DIHED
          WRITE (LUN,233) CL_LSTRP(J), CD_LSTRP(J), CDV_LSTRP(J),
     &                    CNRMSTRP(J), CAXLSTRP(J), 
     &                    CNC(J),    DWWAKE(J),
     &                    CMLE(J),   CMC4(J)
          DO II = 1, NV
            I = I1 + (II-1)
            XM = 0.5*(RV1(1,I)+RV2(1,I))
            YM = 0.5*(RV1(2,I)+RV2(2,I))
            ZM = 0.5*(RV1(3,I)+RV2(3,I))
            WRITE (LUN,234) I,XM,YM,ZM,DXV(I),SLOPEC(I),DCP(I)
          END DO
        END DO
      END DO
      WRITE(LUN,200)
C
      RETURN
C
  200 FORMAT(1X,
     &'---------------------------------------------------------------')
  230 FORMAT (' Vortex Strengths (by surface, by strip)')
  231 FORMAT (/1X,78('*')/2X,'Surface #',I2,5X,A/
     &        5X,'# Chordwise  =',I3,3X,'# Spanwise =',I3,
     &        3X,'First strip  =',I3/
     &        5X,'Surface area =',F12.6,5X,'  Ave. chord =',F12.6)
C
  212 FORMAT (/'  Forces referred to Sref, Cref, Bref ',
     &        'about Xref, Yref, Zref'/
     &         '  ',A)
  213 FORMAT ( 5X,'CLsurf  =',F10.5,5X,'Clsurf  =',F10.5,
     &        /5X,'CYsurf  =',F10.5,5X,'Cmsurf  =',F10.5,
     &        /5X,'CDsurf  =',F10.5,5X,'Cnsurf  =',F10.5, 
     &        /5X,'CDisurf =',F10.5,5x,'CDvsurf =',F10.5)
  214 FORMAT (/'  Forces referred to Ssurf, Cave ',
     &         'about hinge axis thru LE'/
     &         5X,'CLsurf  =',F10.5,5X,'CDsurf  =',F10.5/
     &         1X,78('*'))
C
  232 FORMAT (/1X,'Strip #',I3,5X,'# Chordwise =',I3,
     &         3X,'First Vortex =',I4/
     &         4X,'Xle =',F10.5,4X,'Ave. Chord   =',F10.4,
     &         3X,'Incidence  =',F10.4,' deg'/
     &         4X,'Yle =',F10.5,4X,'Strip Width  =',F10.5,
     &         3X,'Strip Area =',F12.6/
     &         4X,'Zle =',F10.5,4X,'Strip Dihed. =',F10.4)
  233 FORMAT (/4X,'cl  =',F10.5,4X,'   cd  =',F10.5,4X,'  cdv =',F10.5,
     &        /4X,'cn  =',F10.5,4X,'   ca  =',F10.5,
     &         4X,'  cnc =',F10.5,4X,'wake dnwsh =',F10.5,
     &        /4X,'cmLE=',F10.5,4X,'cm c/4 =',F10.5,
     &       //4X,'I',8X,'X   ',8X,'Y   ',8X,'Z   ',8X,'DX  ',
     &         6X,'Slope',8X,'dCp')
  234 FORMAT (2X,I3,6(2X,F10.5))
C
      END ! OUTELE



      SUBROUTINE OUTHINGE(LUN)
C
C...PURPOSE  To print out results of the vortex lattice calculation
C            for the input configuration.  
C
C...INPUT    Configuration data for case in labeled commons
C          
C...OUTPUT   Printed output on logical unit LUN
C
      INCLUDE 'AVL.INC'
C
 1000 FORMAT (A)
C
      IF (LUN.EQ.0) RETURN
C
C...Print out the results
C
C...Hinge moments for each CONTROL
      WRITE(LUN,200)
 200  FORMAT(1X,
     &'---------------------------------------------------------------')
C
      WRITE (LUN,210) SREF,CREF
  210 FORMAT (
     & ' Control Hinge Moments' /
     & ' (referred to    Sref =',G12.4,   3X,'Cref =',F10.4,')' )
C
      WRITE (LUN,212) 
 212  FORMAT(/' Control          Chinge'
     &       /' ---------------- -----------')
C
      DO N = 1, NCONTROL
        WRITE (LUN,220) DNAME(N),CHINGE(N)
  220   FORMAT (1X,A16,G12.4)
      END DO
C
      WRITE(LUN,200)
C
      RETURN
      END ! OUTHINGE



      SUBROUTINE OUTCNC(LUN)
C
C...PURPOSE  To write out a CNC loading file
C            for the input configuration strips
C
C...INPUT    Configuration data for case in labeled commons
C          
C...OUTPUT   Printed output on logical unit LUN
C
      INCLUDE 'AVL.INC'
      CHARACTER*1 ANS
      CHARACTER*80 FNAM
      SAVE FNAM
      DATA FNAM /' '/
C
 1000 FORMAT (A)
C
      IF (LUN.EQ.0) RETURN
C
c      WRITE(*,2090) 
c 2090 FORMAT('Output file Simple(y,cnc,cl) or Full(x,y,z,cnc,cl,c)? ',$)
c      READ (*,1000) ANS
c      CALL TOUPER(ANS)
      ANS = 'F'
C
C...Print out the results -> strip loadings
c     WRITE (LUN,210) 
      DO J=1, NSTRIP
        I = IJFRST(J)
        XM = 0.5*(RV1(1,I)+RV2(1,I))
        YM = 0.5*(RV1(2,I)+RV2(2,I))
        ZM = 0.5*(RV1(3,I)+RV2(3,I))
        CNCM = CNC(J)
        CLM  = CL_LSTRP(J)
        CHM  = CHORD(J)
        DYM  = WSTRIP(J)
        ASM  = DYM*CHM
        IF(ANS.EQ.'S') THEN
          WRITE (LUN,213) YM,CNCM,CLM,CHM
         ELSE
          WRITE (LUN,212) XM,YM,ZM,CNCM,CLM,CHM,DYM,ASM
        ENDIF
      END DO
C
  210 FORMAT (//' *** Strip Loadings')
  212 FORMAT (3(F8.3,1X),2(F10.4,1X),2(F8.4,1X),F9.4)
  213 FORMAT (F8.3,1X,3(F10.4,1X))
C
      RETURN
      END ! OUTCNC



      SUBROUTINE DERMATM(LU)
C---------------------------------------------------------
C     Calculates and outputs stability derivative matrix
C     for current ALFA, BETA.
C---------------------------------------------------------
      INCLUDE 'AVL.INC'
      CHARACTER*50 SATYPE
      REAL WROT_RX(3), WROT_RZ(3), WROT_A(3)
C
      CALL GETSA(LNASA_SA,SATYPE,DIR)
C
C---- set freestream velocity components from alpha, beta
      CALL VINFAB
C
C---- calculate forces and sensitivities
      CALL AERO
C
C---- set stability-axes rates (RX,RY,RZ) in terms of body-axes rates
      CA = COS(ALFA)
      SA = SIN(ALFA)
      RX = (WROT(1)*CA + WROT(3)*SA) * DIR
      RY =  WROT(2)
      RZ = (WROT(3)*CA - WROT(1)*SA) * DIR
C
C---- now vice-versa, and set sensitivities (which is what's really needed)
cc    WROT(1)    =  RX*CA - RZ*SA
cc    WROT(2)    =  RY
cc    WROT(3)    =  RZ*CA + RX*SA
C
      WROT_RX(1) = CA     * DIR
      WROT_RX(2) = 0.
      WROT_RX(3) =     SA * DIR
C
      WROT_RZ(1) =    -SA * DIR
      WROT_RZ(2) = 0.
      WROT_RZ(3) = CA     * DIR
C
      WROT_A(1)  = -RX*SA - RZ*CA   !!! = -WROT(3)
      WROT_A(2)  =  0.
      WROT_A(3)  = -RZ*SA + RX*CA   !!! =  WROT(1)
C
C
C---- set force derivatives in stability axes
      CL_AL = CLTOT_U(1)*VINF_A(1) + CLTOT_U(4)*WROT_A(1)
     &      + CLTOT_U(2)*VINF_A(2) + CLTOT_U(5)*WROT_A(2)
     &      + CLTOT_U(3)*VINF_A(3) + CLTOT_U(6)*WROT_A(3) + CLTOT_A
      CL_BE = CLTOT_U(1)*VINF_B(1)
     &      + CLTOT_U(2)*VINF_B(2)
     &      + CLTOT_U(3)*VINF_B(3)
      CL_RX = CLTOT_U(4)*WROT_RX(1) + CLTOT_U(6)*WROT_RX(3)
      CL_RY = CLTOT_U(5)
      CL_RZ = CLTOT_U(6)*WROT_RZ(3) + CLTOT_U(4)*WROT_RZ(1)
C
      CY_AL = CYTOT_U(1)*VINF_A(1) + CYTOT_U(4)*WROT_A(1)
     &      + CYTOT_U(2)*VINF_A(2) + CYTOT_U(5)*WROT_A(2)
     &      + CYTOT_U(3)*VINF_A(3) + CYTOT_U(6)*WROT_A(3)
      CY_BE = CYTOT_U(1)*VINF_B(1)
     &      + CYTOT_U(2)*VINF_B(2)
     &      + CYTOT_U(3)*VINF_B(3)
      CY_RX = CYTOT_U(4)*WROT_RX(1) + CYTOT_U(6)*WROT_RX(3)
      CY_RY = CYTOT_U(5)
      CY_RZ = CYTOT_U(6)*WROT_RZ(3) + CYTOT_U(4)*WROT_RZ(1)
C
      CR_AL = CRTOT_U(1)*VINF_A(1) + CRTOT_U(4)*WROT_A(1)
     &      + CRTOT_U(2)*VINF_A(2) + CRTOT_U(5)*WROT_A(2)
     &      + CRTOT_U(3)*VINF_A(3) + CRTOT_U(6)*WROT_A(3)
      CR_BE = CRTOT_U(1)*VINF_B(1)
     &      + CRTOT_U(2)*VINF_B(2)
     &      + CRTOT_U(3)*VINF_B(3)
      CR_RX = CRTOT_U(4)*WROT_RX(1) + CRTOT_U(6)*WROT_RX(3)
      CR_RY = CRTOT_U(5)
      CR_RZ = CRTOT_U(6)*WROT_RZ(3) + CRTOT_U(4)*WROT_RZ(1)
C
      CM_AL = CMTOT_U(1)*VINF_A(1) + CMTOT_U(4)*WROT_A(1)
     &      + CMTOT_U(2)*VINF_A(2) + CMTOT_U(5)*WROT_A(2)
     &      + CMTOT_U(3)*VINF_A(3) + CMTOT_U(6)*WROT_A(3)
      CM_BE = CMTOT_U(1)*VINF_B(1)
     &      + CMTOT_U(2)*VINF_B(2)
     &      + CMTOT_U(3)*VINF_B(3)
      CM_RX = CMTOT_U(4)*WROT_RX(1) + CMTOT_U(6)*WROT_RX(3)
      CM_RY = CMTOT_U(5)
      CM_RZ = CMTOT_U(6)*WROT_RZ(3) + CMTOT_U(4)*WROT_RZ(1)
C
      CN_AL = CNTOT_U(1)*VINF_A(1) + CNTOT_U(4)*WROT_A(1)
     &      + CNTOT_U(2)*VINF_A(2) + CNTOT_U(5)*WROT_A(2)
     &      + CNTOT_U(3)*VINF_A(3) + CNTOT_U(6)*WROT_A(3)
      CN_BE = CNTOT_U(1)*VINF_B(1)
     &      + CNTOT_U(2)*VINF_B(2)
     &      + CNTOT_U(3)*VINF_B(3)
      CN_RX = CNTOT_U(4)*WROT_RX(1) + CNTOT_U(6)*WROT_RX(3)
      CN_RY = CNTOT_U(5)
      CN_RZ = CNTOT_U(6)*WROT_RZ(3) + CNTOT_U(4)*WROT_RZ(1)
C
C
      CALL OUTTOT(LU)
C
      WRITE(LU,7004)
 7004 FORMAT(/' Derivatives...')
C
      WRITE(LU,7006)
 7006 FORMAT(14X, 4X,'           alpha',
     &            4X,'            beta'
     &      /14X, 4X,'----------------',
     &            4X,'----------------')
C
      WRITE(LU,7010) CL_AL, CL_BE
 7010 FORMAT(' z force     |','    CLa =',F11.6,'    CLb =',F11.6)
C
      WRITE(LU,7020) CY_AL, CY_BE
 7020 FORMAT(' y force     |','    CYa =',F11.6,'    CYb =',F11.6)
C
      WRITE(LU,7040) DIR*CR_AL, DIR*CR_BE
 7040 FORMAT(' roll  x mom.|','    Cla =',F11.6,'    Clb =',F11.6)
C
      WRITE(LU,7050) CM_AL, CM_BE
 7050 FORMAT(' pitch y mom.|','    Cma =',F11.6,'    Cmb =',F11.6)
C
      WRITE(LU,7060) DIR*CN_AL, DIR*CN_BE
 7060 FORMAT(' yaw   z mom.|','    Cna =',F11.6,'    Cnb =',F11.6)
C
C
      WRITE(LU,7106)
 7106 FORMAT(/14X, 4X,'    roll rate  p',
     &             4X,'   pitch rate  q',
     &             4X,'     yaw rate  r'
     &       /14X, 4X,'----------------',
     &             4X,'----------------',
     &             4X,'----------------' )
C
      WRITE(LU,7110) CL_RX*2.0/BREF, 
     &               CL_RY*2.0/CREF, 
     &               CL_RZ*2.0/BREF
 7110 FORMAT(' z force     |','    CLp =',F11.6,
     &                        '    CLq =',F11.6,
     &                        '    CLr =',F11.6 )
C
      WRITE(LU,7120) CY_RX*2.0/BREF,
     &               CY_RY*2.0/CREF,
     &               CY_RZ*2.0/BREF
 7120 FORMAT(' y force     |','    CYp =',F11.6,
     &                        '    CYq =',F11.6,
     &                        '    CYr =',F11.6 )
C
      WRITE(LU,7140) DIR*CR_RX*2.0/BREF, 
     &               DIR*CR_RY*2.0/CREF,
     &               DIR*CR_RZ*2.0/BREF
 7140 FORMAT(' roll  x mom.|','    Clp =',F11.6,
     &                        '    Clq =',F11.6,
     &                        '    Clr =',F11.6 )
C
      WRITE(LU,7150) CM_RX*2.0/BREF,
     &               CM_RY*2.0/CREF,
     &               CM_RZ*2.0/BREF
 7150 FORMAT(' pitch y mom.|','    Cmp =',F11.6,
     &                        '    Cmq =',F11.6,
     &                        '    Cmr =',F11.6 )
C
      WRITE(LU,7160) DIR*CN_RX*2.0/BREF,
     &               DIR*CN_RY*2.0/CREF,
     &               DIR*CN_RZ*2.0/BREF
 7160 FORMAT(' yaw   z mom.|','    Cnp =',F11.6,
     &                        '    Cnq =',F11.6,
     &                        '    Cnr =',F11.6 )
C
      IF(NCONTROL.GT.0) THEN
C
      WRITE(LU,8106) (DNAME(K), K, K=1, NCONTROL)
 8106 FORMAT(/14X,20(4X,A12, ' d',I1,' '))
      WRITE(LU,8107) (' ',K=1, NCONTROL)
 8107 FORMAT( 14X,20(3X,A,'----------------'))
C
      WRITE(LU,8110) (' ',K,CLTOT_D(K), K=1, NCONTROL)
 8110 FORMAT(' z force     |',20(A,'  CLd',I1,' =',F11.6))
C
      WRITE(LU,8120) (' ',K,CYTOT_D(K), K=1, NCONTROL)
 8120 FORMAT(' y force     |',20(A,'  CYd',I1,' =',F11.6))
C
      WRITE(LU,8140) (' ',K,DIR*CRTOT_D(K), K=1, NCONTROL)
 8140 FORMAT(' roll  x mom.|',20(A,'  Cld',I1,' =',F11.6))
C
      WRITE(LU,8150) (' ',K,    CMTOT_D(K), K=1, NCONTROL)
 8150 FORMAT(' pitch y mom.|',20(A,'  Cmd',I1,' =',F11.6))
C
      WRITE(LU,8160) (' ',K,DIR*CNTOT_D(K), K=1, NCONTROL)
 8160 FORMAT(' yaw   z mom.|',20(A,'  Cnd',I1,' =',F11.6))
C
      WRITE(LU,8170) (' ',K,    CDFF_D(K), K=1, NCONTROL)
 8170 FORMAT(' Trefftz drag|',20(A,'CDffd',I1,' =',F11.6))
C
      WRITE(LU,8180) (' ',K,  SPANEF_D(K), K=1, NCONTROL)
 8180 FORMAT(' span eff.   |',20(A,'   ed',I1,' =',F11.6))
C
      WRITE(LU,*)
      WRITE(LU,*)
C
      ENDIF
C
      IF(NDESIGN.GT.0) THEN
C
      WRITE(LU,8206) (GNAME(K), K, K=1, NDESIGN)
 8206 FORMAT(/14X,20(4X,A12, ' g',I1,' '))
      WRITE(LU,8207) (' ',K=1, NDESIGN)
 8207 FORMAT( 14X,20(3X,A,'----------------'))
C
      WRITE(LU,8210) (' ',K,CLTOT_G(K), K=1, NDESIGN)
 8210 FORMAT(' z force     |',20(A,'  CLg',I1,' =',F11.6))
C
      WRITE(LU,8220) (' ',K,CYTOT_G(K), K=1, NDESIGN)
 8220 FORMAT(' y force     |',20(A,'  CYg',I1,' =',F11.6))
C
      WRITE(LU,8230) (' ',K,DIR*CRTOT_G(K), K=1, NDESIGN)
 8230 FORMAT(' roll  x mom.|',20(A,'  Clg',I1,' =',F11.6))
C
      WRITE(LU,8240) (' ',K,    CMTOT_G(K), K=1, NDESIGN)
 8240 FORMAT(' pitch y mom.|',20(A,'  Cmg',I1,' =',F11.6))
C
      WRITE(LU,8250) (' ',K,DIR*CNTOT_G(K), K=1, NDESIGN)
 8250 FORMAT(' yaw   z mom.|',20(A,'  Cng',I1,' =',F11.6))
C
      WRITE(LU,8260) (' ',K,    CDFF_G(K), K=1, NDESIGN)
 8260 FORMAT(' Trefftz drag|',20(A,'CDffg',I1,' =',F11.6))
C
      WRITE(LU,8270) (' ',K,  SPANEF_G(K), K=1, NDESIGN)
 8270 FORMAT(' span eff.   |',20(A,'   eg',I1,' =',F11.6))
C
      WRITE(LU,*)
      WRITE(LU,*)
C
      ENDIF
C
      IF(CL_AL .NE. 0.0) THEN
       XNP = XYZREF(1) - CREF*CM_AL/CL_AL
       WRITE(LU,8401) XNP
 8401  FORMAT(/' Neutral point  Xnp =', F11.6)
      ENDIF
C
      IF(ABS(CR_RZ*CN_BE) .GT. 0.0001) THEN
       BB = CR_BE*CN_RZ / (CR_RZ*CN_BE)
       WRITE(LU,8402) BB 
 8402  FORMAT(/' Clb Cnr / Clr Cnb  =', F11.6,
     &    '    (  > 1 if spirally stable )')
      ENDIF

C
      RETURN
      END ! DERMATM



      SUBROUTINE DERMATS(LU)
C---------------------------------------------------------
C     Calculates and outputs stability derivative matrix
C     for current ALFA, BETA.
C---------------------------------------------------------
      INCLUDE 'AVL.INC'
      CHARACTER*50 SATYPE
      REAL WROT_RX(3), WROT_RZ(3), WROT_A(3)
      REAL
     & CRSAX_U(NUMAX),CMSAX_U(NUMAX),CNSAX_U(NUMAX),
     & CRSAX_D(NDMAX),CMSAX_D(NDMAX),CNSAX_D(NDMAX),
     & CRSAX_G(NGMAX),CMSAX_G(NGMAX),CNSAX_G(NGMAX)
C
      CALL GETSA(LNASA_SA,SATYPE,DIR)
C
C---- set freestream velocity components from alpha, beta
      CALL VINFAB
C
C---- calculate forces and sensitivities
      CALL AERO
C
C---- set stability-axes rates (RX,RY,RZ) in terms of body-axes rates
      CA = COS(ALFA)
      SA = SIN(ALFA)
C
      RX = (WROT(1)*CA + WROT(3)*SA) * DIR
      RY =  WROT(2)
      RZ = (WROT(3)*CA - WROT(1)*SA) * DIR
C
C---- now vice-versa, and set sensitivities (which is what's really needed)
cc    WROT(1)    =  RX*CA - RZ*SA
cc    WROT(2)    =  RY
cc    WROT(3)    =  RZ*CA + RX*SA
C
      WROT_RX(1) = CA     * DIR
      WROT_RX(2) = 0.
      WROT_RX(3) =     SA * DIR
C
      WROT_RZ(1) =    -SA * DIR
      WROT_RZ(2) = 0.
      WROT_RZ(3) = CA     * DIR
C
      WROT_A(1)  = -RX*SA - RZ*CA   !!! = -WROT(3)
      WROT_A(2)  =  0.
      WROT_A(3)  = -RZ*SA + RX*CA   !!! =  WROT(1)
C
C
      CRSAX = CRTOT*CA + CNTOT*SA
      CMSAX = CMTOT              
      CNSAX = CNTOT*CA - CRTOT*SA  
      CRSAX_A = -CRTOT*SA + CNTOT*CA
      CNSAX_A = -CNTOT*SA - CRTOT*CA
C
      DO K = 1, 6
        CRSAX_U(K) = CRTOT_U(K)*CA + CNTOT_U(K)*SA
        CMSAX_U(K) = CMTOT_U(K)              
        CNSAX_U(K) = CNTOT_U(K)*CA - CRTOT_U(K)*SA  
      ENDDO
      DO K = 1, NCONTROL
        CRSAX_D(K) = CRTOT_D(K)*CA + CNTOT_D(K)*SA
        CMSAX_D(K) = CMTOT_D(K)              
        CNSAX_D(K) = CNTOT_D(K)*CA - CRTOT_D(K)*SA  
      ENDDO
      DO K = 1, NDESIGN
        CRSAX_G(K) = CRTOT_G(K)*CA + CNTOT_G(K)*SA
        CMSAX_G(K) = CMTOT_G(K)              
        CNSAX_G(K) = CNTOT_G(K)*CA - CRTOT_G(K)*SA  
      ENDDO

C
C---- set force derivatives in stability axes
      CL_AL = CLTOT_U(1)*VINF_A(1) + CLTOT_U(4)*WROT_A(1)
     &      + CLTOT_U(2)*VINF_A(2) + CLTOT_U(5)*WROT_A(2)
     &      + CLTOT_U(3)*VINF_A(3) + CLTOT_U(6)*WROT_A(3) + CLTOT_A
      CL_BE = CLTOT_U(1)*VINF_B(1)
     &      + CLTOT_U(2)*VINF_B(2)
     &      + CLTOT_U(3)*VINF_B(3)
      CL_RX = CLTOT_U(4)*WROT_RX(1) + CLTOT_U(6)*WROT_RX(3)
      CL_RY = CLTOT_U(5)
      CL_RZ = CLTOT_U(6)*WROT_RZ(3) + CLTOT_U(4)*WROT_RZ(1)
C
      CY_AL = CYTOT_U(1)*VINF_A(1) + CYTOT_U(4)*WROT_A(1)
     &      + CYTOT_U(2)*VINF_A(2) + CYTOT_U(5)*WROT_A(2)
     &      + CYTOT_U(3)*VINF_A(3) + CYTOT_U(6)*WROT_A(3)
      CY_BE = CYTOT_U(1)*VINF_B(1)
     &      + CYTOT_U(2)*VINF_B(2)
     &      + CYTOT_U(3)*VINF_B(3)
      CY_RX = CYTOT_U(4)*WROT_RX(1) + CYTOT_U(6)*WROT_RX(3)
      CY_RY = CYTOT_U(5)
      CY_RZ = CYTOT_U(6)*WROT_RZ(3) + CYTOT_U(4)*WROT_RZ(1)
C
      CR_AL = CRSAX_U(1)*VINF_A(1) + CRSAX_U(4)*WROT_A(1)
     &      + CRSAX_U(2)*VINF_A(2) + CRSAX_U(5)*WROT_A(2)
     &      + CRSAX_U(3)*VINF_A(3) + CRSAX_U(6)*WROT_A(3) + CRSAX_A
      CR_BE = CRSAX_U(1)*VINF_B(1)
     &      + CRSAX_U(2)*VINF_B(2)
     &      + CRSAX_U(3)*VINF_B(3)
      CR_RX = CRSAX_U(4)*WROT_RX(1) + CRSAX_U(6)*WROT_RX(3)
      CR_RY = CRSAX_U(5)
      CR_RZ = CRSAX_U(6)*WROT_RZ(3) + CRSAX_U(4)*WROT_RZ(1)
C
      CM_AL = CMSAX_U(1)*VINF_A(1) + CMSAX_U(4)*WROT_A(1)
     &      + CMSAX_U(2)*VINF_A(2) + CMSAX_U(5)*WROT_A(2)
     &      + CMSAX_U(3)*VINF_A(3) + CMSAX_U(6)*WROT_A(3)
      CM_BE = CMSAX_U(1)*VINF_B(1)
     &      + CMSAX_U(2)*VINF_B(2)
     &      + CMSAX_U(3)*VINF_B(3)
      CM_RX = CMSAX_U(4)*WROT_RX(1) + CMSAX_U(6)*WROT_RX(3)
      CM_RY = CMSAX_U(5)
      CM_RZ = CMSAX_U(6)*WROT_RZ(3) + CMSAX_U(4)*WROT_RZ(1)
C
      CN_AL = CNSAX_U(1)*VINF_A(1) + CNSAX_U(4)*WROT_A(1)
     &      + CNSAX_U(2)*VINF_A(2) + CNSAX_U(5)*WROT_A(2)
     &      + CNSAX_U(3)*VINF_A(3) + CNSAX_U(6)*WROT_A(3) + CNSAX_A
      CN_BE = CNSAX_U(1)*VINF_B(1)
     &      + CNSAX_U(2)*VINF_B(2)
     &      + CNSAX_U(3)*VINF_B(3)
      CN_RX = CNSAX_U(4)*WROT_RX(1) + CNSAX_U(6)*WROT_RX(3)
      CN_RY = CNSAX_U(5)
      CN_RZ = CNSAX_U(6)*WROT_RZ(3) + CNSAX_U(4)*WROT_RZ(1)
C
C
      CALL OUTTOT(LU)
C
      WRITE(LU,7004)
 7004 FORMAT(/' Stability-axis derivatives...')
C
      WRITE(LU,7006)
 7006 FORMAT(/14X, 4X,'           alpha',
     &             4X,'            beta'
     &       /14X, 4X,'----------------',
     &             4X,'----------------')
C
      WRITE(LU,7010) CL_AL, CL_BE
 7010 FORMAT(' z'' force CL |' ,'    CLa =',F11.6,'    CLb =',F11.6)
C
      WRITE(LU,7020) CY_AL, CY_BE
 7020 FORMAT(' y  force CY |'  ,'    CYa =',F11.6,'    CYb =',F11.6)
C
      WRITE(LU,7040) DIR*CR_AL, DIR*CR_BE
 7040 FORMAT(' x'' mom.  Cl''|','    Cla =',F11.6,'    Clb =',F11.6)
C
      WRITE(LU,7050) CM_AL, CM_BE
 7050 FORMAT(' y  mom.  Cm |'  ,'    Cma =',F11.6,'    Cmb =',F11.6)
C
      WRITE(LU,7060) DIR*CN_AL, DIR*CN_BE
 7060 FORMAT(' z'' mom.  Cn''|','    Cna =',F11.6,'    Cnb =',F11.6)
C
C
      WRITE(LU,7106)
 7106 FORMAT(/14X, 4X,'   roll rate  p''',
     &             4X,'  pitch rate  q''',
     &             4X,'    yaw rate  r'''
     &       /14X, 4X,'----------------',
     &             4X,'----------------',
     &             4X,'----------------' )
C
      WRITE(LU,7110) CL_RX*2.0/BREF, 
     &               CL_RY*2.0/CREF, 
     &               CL_RZ*2.0/BREF
 7110 FORMAT(' z'' force CL |','    CLp =',F11.6,
     &                         '    CLq =',F11.6,
     &                         '    CLr =',F11.6 )
C
      WRITE(LU,7120) CY_RX*2.0/BREF,
     &               CY_RY*2.0/CREF,
     &               CY_RZ*2.0/BREF
 7120 FORMAT(' y  force CY |','    CYp =',F11.6,
     &                        '    CYq =',F11.6,
     &                        '    CYr =',F11.6 )
C
      WRITE(LU,7140) DIR*CR_RX*2.0/BREF, 
     &               DIR*CR_RY*2.0/CREF,
     &               DIR*CR_RZ*2.0/BREF
 7140 FORMAT(' x'' mom.  Cl''|','    Clp =',F11.6,
     &                        '    Clq =',F11.6,
     &                        '    Clr =',F11.6 )
C
      WRITE(LU,7150) CM_RX*2.0/BREF,
     &               CM_RY*2.0/CREF,
     &               CM_RZ*2.0/BREF
 7150 FORMAT(' y  mom.  Cm |','    Cmp =',F11.6,
     &                        '    Cmq =',F11.6,
     &                        '    Cmr =',F11.6 )
C
      WRITE(LU,7160) DIR*CN_RX*2.0/BREF,
     &               DIR*CN_RY*2.0/CREF,
     &               DIR*CN_RZ*2.0/BREF
 7160 FORMAT(' z'' mom.  Cn''|','    Cnp =',F11.6,
     &                        '    Cnq =',F11.6,
     &                        '    Cnr =',F11.6 )
C
      IF(NCONTROL.GT.0) THEN
C
      WRITE(LU,8106) (DNAME(K), K, K=1, NCONTROL)
 8106 FORMAT(/14X,20(4X,A12, ' d',I1,' '))
      WRITE(LU,8107) (' ',K=1, NCONTROL)
 8107 FORMAT( 14X,20(3X,A,'----------------'))
C
      WRITE(LU,8110) (' ',K,CLTOT_D(K), K=1, NCONTROL)
 8110 FORMAT(' z'' force CL |' ,20(A,'  CLd',I1,' =',F11.6))
C
      WRITE(LU,8120) (' ',K,CYTOT_D(K), K=1, NCONTROL)
 8120 FORMAT(' y  force CY |'  ,20(A,'  CYd',I1,' =',F11.6))
C
      WRITE(LU,8140) (' ',K,DIR*CRTOT_D(K), K=1, NCONTROL)
 8140 FORMAT(' x'' mom.  Cl''|',20(A,'  Cld',I1,' =',F11.6))
C
      WRITE(LU,8150) (' ',K,    CMTOT_D(K), K=1, NCONTROL)
 8150 FORMAT(' y  mom.  Cm |'  ,20(A,'  Cmd',I1,' =',F11.6))
C
      WRITE(LU,8160) (' ',K,DIR*CNTOT_D(K), K=1, NCONTROL)
 8160 FORMAT(' z'' mom.  Cn''|',20(A,'  Cnd',I1,' =',F11.6))
C
      WRITE(LU,8170) (' ',K,    CDFF_D(K), K=1, NCONTROL)
 8170 FORMAT(' Trefftz drag|'  ,20(A,'CDffd',I1,' =',F11.6))
C
      WRITE(LU,8180) (' ',K,  SPANEF_D(K), K=1, NCONTROL)
 8180 FORMAT(' span eff.   |'  ,20(A,'   ed',I1,' =',F11.6))
C
      WRITE(LU,*)
      WRITE(LU,*)
C
      ENDIF
C
      IF(NDESIGN.GT.0) THEN
C
      WRITE(LU,8206) (GNAME(K), K, K=1, NDESIGN)
 8206 FORMAT(/14X,20(4X,A12, ' g',I1,' '))
      WRITE(LU,8207) (' ',K=1, NDESIGN)
 8207 FORMAT( 14X,20(3X,A,'----------------'))
C
      WRITE(LU,8210) (' ',K,CLTOT_G(K), K=1, NDESIGN)
 8210 FORMAT(' z'' force CL |'  ,20(A,'  CLg',I1,' =',F11.6))
C
      WRITE(LU,8220) (' ',K,CYTOT_G(K), K=1, NDESIGN)
 8220 FORMAT(' y  force CY |'   ,20(A,'  CYg',I1,' =',F11.6))
C
      WRITE(LU,8230) (' ',K,DIR*CRTOT_G(K), K=1, NDESIGN)
 8230 FORMAT(' x'' mom.  Cl''|' ,20(A,'  Clg',I1,' =',F11.6))
C
      WRITE(LU,8240) (' ',K,    CMTOT_G(K), K=1, NDESIGN)
 8240 FORMAT(' y  mom.  Cm |'    ,20(A,'  Cmg',I1,' =',F11.6))
C
      WRITE(LU,8250) (' ',K,DIR*CNTOT_G(K), K=1, NDESIGN)
 8250 FORMAT(' z'' mom.  Cn''|',20(A,'  Cng',I1,' =',F11.6))
C
      WRITE(LU,8260) (' ',K,    CDFF_G(K), K=1, NDESIGN)
 8260 FORMAT(' Trefftz drag|',20(A,'CDffg',I1,' =',F11.6))
C
      WRITE(LU,8270) (' ',K,  SPANEF_G(K), K=1, NDESIGN)
 8270 FORMAT(' span eff.   |',20(A,'   eg',I1,' =',F11.6))
C
      WRITE(LU,*)
      WRITE(LU,*)
C
      ENDIF
C
      IF(CL_AL .NE. 0.0) THEN
       XNP = XYZREF(1) - CREF*CM_AL/CL_AL
       WRITE(LU,8401) XNP
 8401  FORMAT(/' Neutral point  Xnp =', F11.6)
      ENDIF
C
      IF(ABS(CR_RZ*CN_BE) .GT. 0.0001) THEN
       BB = CR_BE*CN_RZ / (CR_RZ*CN_BE)
       WRITE(LU,8402) BB 
 8402  FORMAT(/' Clb Cnr / Clr Cnb  =', F11.6,
     &    '    (  > 1 if spirally stable )')
      ENDIF

C
      RETURN
      END ! DERMATS



      SUBROUTINE DERMATB(LU)
C---------------------------------------------------------
C     Calculates and outputs stability derivative matrix
C     in body axes
C---------------------------------------------------------
      INCLUDE 'AVL.INC'
      CHARACTER*50 SATYPE
      REAL WROT_RX(3), WROT_RZ(3), WROT_A(3)
C
      CALL GETSA(LNASA_SA,SATYPE,DIR)
C
C---- set freestream velocity components from alpha, beta
      CALL VINFAB
C
C---- calculate forces and sensitivities
      CALL AERO
C
      CALL OUTTOT(LU)
C
      WRITE(LU,7004)
 7004 FORMAT(/' Geometry-axis derivatives...')
C
C
      WRITE(LU,7006)
 7006 FORMAT(/14X, 4X,'  axial   vel. u',
     &             4X,' sideslip vel. v',
     &             4X,'  normal  vel. w'
     &       /14X, 4X,'----------------',
     &             4X,'----------------',
     &             4X,'----------------' )
C
      WRITE(LU,7010) -    CXTOT_U(1),
     &               -DIR*CXTOT_U(2),
     &               -    CXTOT_U(3)
 7010 FORMAT(' x force CX  |','    CXu =',F11.6,
     &                        '    CXv =',F11.6,
     &                        '    CXw =',F11.6 )
C
      WRITE(LU,7020) -DIR*CYTOT_U(1),
     &               -    CYTOT_U(2),
     &               -DIR*CYTOT_U(3)
 7020 FORMAT(' y force CY  |','    CYu =',F11.6,
     &                        '    CYv =',F11.6,
     &                        '    CYw =',F11.6 )
C
      WRITE(LU,7030) -    CZTOT_U(1),
     &               -DIR*CZTOT_U(2),
     &               -    CZTOT_U(3)
 7030 FORMAT(' z force CZ  |','    CZu =',F11.6,
     &                        '    CZv =',F11.6,
     &                        '    CZw =',F11.6 )
C
      WRITE(LU,7040) -    CRTOT_U(1),
     &               -DIR*CRTOT_U(2),
     &               -    CRTOT_U(3)
 7040 FORMAT(' x mom.  Cl  |','    Clu =',F11.6,
     &                        '    Clv =',F11.6,
     &                        '    Clw =',F11.6 )
C
      WRITE(LU,7050) -DIR*CMTOT_U(1),
     &               -    CMTOT_U(2),
     &               -DIR*CMTOT_U(3)
 7050 FORMAT(' y mom.  Cm  |','    Cmu =',F11.6,
     &                        '    Cmv =',F11.6,
     &                        '    Cmw =',F11.6 )
C
      WRITE(LU,7060) -    CNTOT_U(1),
     &               -DIR*CNTOT_U(2),
     &               -    CNTOT_U(3)
 7060 FORMAT(' z mom.  Cn  |','    Cnu =',F11.6,
     &                        '    Cnv =',F11.6,
     &                        '    Cnw =',F11.6 )
C
C
      WRITE(LU,7106)
 7106 FORMAT(/14X, 4X,'    roll rate  p',
     &             4X,'   pitch rate  q',
     &             4X,'     yaw rate  r'
     &       /14X, 4X,'----------------',
     &             4X,'----------------',
     &             4X,'----------------' )
C
      WRITE(LU,7110)     CXTOT_U(4)*2.0/BREF, 
     &               DIR*CXTOT_U(5)*2.0/CREF, 
     &                   CXTOT_U(6)*2.0/BREF
 7110 FORMAT(' x force CX  |','    CXp =',F11.6,
     &                        '    CXq =',F11.6,
     &                        '    CXr =',F11.6 )
C
      WRITE(LU,7120) DIR*CYTOT_U(4)*2.0/BREF,
     &                   CYTOT_U(5)*2.0/CREF,
     &               DIR*CYTOT_U(6)*2.0/BREF
 7120 FORMAT(' y force CY  |','    CYp =',F11.6,
     &                        '    CYq =',F11.6,
     &                        '    CYr =',F11.6 )
C
      WRITE(LU,7130)     CZTOT_U(4)*2.0/BREF,
     &               DIR*CZTOT_U(5)*2.0/CREF,
     &                   CZTOT_U(6)*2.0/BREF
 7130 FORMAT(' z force CZ  |','    CZp =',F11.6,
     &                        '    CZq =',F11.6,
     &                        '    CZr =',F11.6 )
C
      WRITE(LU,7140)     CRTOT_U(4)*2.0/BREF, 
     &               DIR*CRTOT_U(5)*2.0/CREF,
     &                   CRTOT_U(6)*2.0/BREF
 7140 FORMAT(' x mom.  Cl  |','    Clp =',F11.6,
     &                        '    Clq =',F11.6,
     &                        '    Clr =',F11.6 )
C
      WRITE(LU,7150) DIR*CMTOT_U(4)*2.0/BREF,
     &                   CMTOT_U(5)*2.0/CREF,
     &               DIR*CMTOT_U(6)*2.0/BREF
 7150 FORMAT(' y mom.  Cm  |','    Cmp =',F11.6,
     &                        '    Cmq =',F11.6,
     &                        '    Cmr =',F11.6 )
C
      WRITE(LU,7160)     CNTOT_U(4)*2.0/BREF,
     &               DIR*CNTOT_U(5)*2.0/CREF,
     &                   CNTOT_U(6)*2.0/BREF
 7160 FORMAT(' z mom.  Cn  |','    Cnp =',F11.6,
     &                        '    Cnq =',F11.6,
     &                        '    Cnr =',F11.6 )
C
      IF(NCONTROL.GT.0) THEN
C
      WRITE(LU,8106) (DNAME(K), K, K=1, NCONTROL)
 8106 FORMAT(/14X,20(4X,A12, ' d',I1,' '))
      WRITE(LU,8107) (' ',K=1, NCONTROL)
 8107 FORMAT( 14X,20(3X,A,'----------------'))
C
      WRITE(LU,8110) (' ',K,DIR*CXTOT_D(K), K=1, NCONTROL)
 8110 FORMAT(' x force CX  |',20(A,'  CXd',I1,' =',F11.6))
C
      WRITE(LU,8120) (' ',K,    CYTOT_D(K), K=1, NCONTROL)
 8120 FORMAT(' y force CY  |',20(A,'  CYd',I1,' =',F11.6))
C
      WRITE(LU,8130) (' ',K,DIR*CZTOT_D(K), K=1, NCONTROL)
 8130 FORMAT(' z force CZ  |',20(A,'  CZd',I1,' =',F11.6))
C
      WRITE(LU,8140) (' ',K,DIR*CRTOT_D(K), K=1, NCONTROL)
 8140 FORMAT(' x mom.  Cl  |',20(A,'  Cld',I1,' =',F11.6))
C
      WRITE(LU,8150) (' ',K,    CMTOT_D(K), K=1, NCONTROL)
 8150 FORMAT(' y mom.  Cm  |',20(A,'  Cmd',I1,' =',F11.6))
C
      WRITE(LU,8160) (' ',K,DIR*CNTOT_D(K), K=1, NCONTROL)
 8160 FORMAT(' z mom.  Cn  |',20(A,'  Cnd',I1,' =',F11.6))
C
      WRITE(LU,*)
      WRITE(LU,*)
C
      ENDIF
C
      IF(NDESIGN.GT.0) THEN
C
      WRITE(LU,8206) (GNAME(K), K, K=1, NDESIGN)
 8206 FORMAT(/14X,20(4X,A12, ' g',I1,' '))
      WRITE(LU,8207) (' ',K=1, NDESIGN)
 8207 FORMAT( 14X,20(3X,A,'----------------'))
C
      WRITE(LU,8210) (' ',K,DIR*CXTOT_G(K), K=1, NDESIGN)
 8210 FORMAT(' x force CX  |',20(A,'  CXg',I1,' =',F11.6))
C
      WRITE(LU,8220) (' ',K,    CYTOT_G(K), K=1, NDESIGN)
 8220 FORMAT(' y force CY  |',20(A,'  CYg',I1,' =',F11.6))
C
      WRITE(LU,8230) (' ',K,DIR*CZTOT_G(K), K=1, NDESIGN)
 8230 FORMAT(' z force CZ  |',20(A,'  CZg',I1,' =',F11.6))
C
      WRITE(LU,8240) (' ',K,DIR*CRTOT_G(K), K=1, NDESIGN)
 8240 FORMAT(' x mom.  Cl  |',20(A,'  Clg',I1,' =',F11.6))
C
      WRITE(LU,8250) (' ',K,    CMTOT_G(K), K=1, NDESIGN)
 8250 FORMAT(' y mom.  Cm  |',20(A,'  Cmg',I1,' =',F11.6))
C
      WRITE(LU,8260) (' ',K,DIR*CNTOT_G(K), K=1, NDESIGN)
 8260 FORMAT(' z mom.  Cn  |',20(A,'  Cng',I1,' =',F11.6))
C
      WRITE(LU,*)
      WRITE(LU,*)
C
      ENDIF
C
      RETURN
      END ! DERMATB



      SUBROUTINE DUMPIT(LU,NF,VINF,ALPHA, BETA,
     &                  OMEGAX, OMEGAY, OMEGAZ,
     &                  CFX, CFY, CFZ, CMX, CMY, CMZ)
C
C--- Writes flow condition header to logical unit
C
       REAL VINF(NF),ALPHA(NF), BETA(NF),
     &     OMEGAX(NF), OMEGAY(NF), OMEGAZ(NF),
     &     CFX(NF), CFY(NF), CFZ(NF), CMX(NF), CMY(NF), CMZ(NF)
C
      DO IF=1, NF
        WRITE(LU,2050) IF, VINF(IF)
        WRITE(LU,2060) ALPHA(IF), BETA(IF),
     &                 OMEGAX(IF), OMEGAY(IF), OMEGAZ(IF)
        WRITE(LU,2070) CFX(IF), CFY(IF), CFZ(IF),
     &                 CMX(IF), CMY(IF), CMZ(IF)
      END DO
C
 2050 FORMAT(/1X,'Flow condition', I3, '    Vinf =', F8.3)
 2060 FORMAT(/1X,6X,'Alpha' ,7X,'Beta',
     &           5X,'Omegax',5X,'Omegay',5X,'Omegaz' /1X,5F11.6)
 2070 FORMAT(/1X,8X,'CFx',8X,'CFy',8X,'CFz',
     &           8X,'CMx',8X,'CMy',8X,'CMz' /1X,6F11.6 / )
C
      RETURN
      END


      SUBROUTINE GETSA(LSA,SATYPE,DIR)
      LOGICAL LSA
      CHARACTER*(*) SATYPE
C
      IF(LSA) THEN
       SATYPE = 'Standard axis orientation,  X fwd, Z down'
       DIR = -1.0
      ELSE
       SATYPE = 'Geometric axis orientation,  X aft, Z up  '
       DIR =  1.0
      ENDIF
C
      RETURN
      END ! GETSA
