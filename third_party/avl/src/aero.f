C***********************************************************************
C    Module:  aero.f
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

      SUBROUTINE AERO
      INCLUDE 'AVL.INC'
C
      CDTOT = 0.
      CLTOT = 0.
      CXTOT = 0.
      CYTOT = 0.
      CZTOT = 0.
      CRTOT = 0.
      CMTOT = 0.
      CNTOT = 0.
      CDVTOT = 0.
C
      CDTOT_A = 0.
      CLTOT_A = 0.
C
      DO L = 1, NCONTROL
        CHINGE(L) = 0.
      ENDDO
C
      DO N=1, NUMAX
        CDTOT_U(N) = 0.
        CLTOT_U(N) = 0.
        CXTOT_U(N) = 0.
        CYTOT_U(N) = 0.
        CZTOT_U(N) = 0.
        CRTOT_U(N) = 0.
        CMTOT_U(N) = 0.
        CNTOT_U(N) = 0.
        DO L = 1, NCONTROL
          CHINGE_U(L,N) = 0.
        ENDDO
      ENDDO
C
      DO N=1, NCONTROL
        CDTOT_D(N) = 0.
        CLTOT_D(N) = 0.
        CXTOT_D(N) = 0.
        CYTOT_D(N) = 0.
        CZTOT_D(N) = 0.
        CRTOT_D(N) = 0.
        CMTOT_D(N) = 0.
        CNTOT_D(N) = 0.
        DO L = 1, NCONTROL
          CHINGE_D(L,N) = 0.
        ENDDO
      ENDDO
C
      DO N=1, NDESIGN
        CDTOT_G(N) = 0.
        CLTOT_G(N) = 0.
        CXTOT_G(N) = 0.
        CYTOT_G(N) = 0.
        CZTOT_G(N) = 0.
        CRTOT_G(N) = 0.
        CMTOT_G(N) = 0.
        CNTOT_G(N) = 0.
        DO L = 1, NCONTROL
          CHINGE_G(L,N) = 0.
        ENDDO
      ENDDO

C
      CALL SFFORC
      CALL BDFORC
      CALL TPFORC
C
C---------------------------------------------------------
C---- add baseline reference CD
c      SINA = SIN(ALFA)
c      COSA = COS(ALFA)
C
      VSQ = VINF(1)**2 + VINF(2)**2 + VINF(3)**2
      VMAG = SQRT(VSQ)
C
      CDVTOT = CDVTOT + CDREF*VSQ
C
      CDTOT = CDTOT + CDREF*VSQ
      CXTOT = CXTOT + CDREF*VINF(1)*VMAG
      CYTOT = CYTOT + CDREF*VINF(2)*VMAG
      CZTOT = CZTOT + CDREF*VINF(3)*VMAG
C
      CXTOT_U(1) = CXTOT_U(1) + CDREF*VMAG
      CYTOT_U(2) = CYTOT_U(2) + CDREF*VMAG
      CZTOT_U(3) = CZTOT_U(3) + CDREF*VMAG
      DO IU = 1, 3
        CDTOT_U(IU) = CDTOT_U(IU) + CDREF*    2.0*VINF(IU)
        CXTOT_U(IU) = CXTOT_U(IU) + CDREF*VINF(1)*VINF(IU)/VMAG
        CYTOT_U(IU) = CYTOT_U(IU) + CDREF*VINF(2)*VINF(IU)/VMAG
        CZTOT_U(IU) = CZTOT_U(IU) + CDREF*VINF(3)*VINF(IU)/VMAG
      ENDDO
C
      RETURN
      END ! AERO



      SUBROUTINE SFFORC
C
C...PURPOSE  To calculate the forces on the configuration,
C            by vortex, strip and surface.
C
C...INPUT    Global Data in labelled commons, defining configuration
C            ALFA       Angle of attack (for stability-axis definition)
C            VINF()     Freestream velocity components
C            WROT()     Roll,Pitch,Yaw  rates
C            MACH       Mach number
C            NVOR       Number of vortices
C            R1         Coordinates of endpoint #1 of bound vortex
C            R2         Coordinates of endpoint #2 of bound vortex
C            ENV        Normal vector at bound vortex midpoint
C            DX         X-length of vortex lattice panel
C            NVSTRP     No. of vortices in strip
C          
C...OUTPUT   DCP                   Vortex element loadings
C            CXYZTOT                Total force,moment coefficients
C            CDFF                  Far-field drag (Trefftz plane)
C            CxxSURF               Surface force,moment coefficients
C            CxxSTRP               Strip force coefficients
C            CNC                   Span load for each strip
C
C...COMMENTS   
C
      INCLUDE 'AVL.INC'
C
      REAL RROT(3)
      REAL VEFF(3)    , VROT(3)  ,
     &     VEFF_U(3,6), VROT_U(3), WROT_U(3)
      REAL VPERP(3)
      REAL G(3), R(3), RH(3), MH(3)
      REAL F(3), F_U(3,6)
      REAL FGAM(3), FGAM_U(3,6), FGAM_D(3,NDMAX), FGAM_G(3,NGMAX)
      REAL ENAVE(3), SPN(3), UDRAG(3), ULIFT(3)
C
      REAL CFX_U(NUMAX), CFY_U(NUMAX), CFZ_U(NUMAX),
     &     CMX_U(NUMAX), CMY_U(NUMAX), CMZ_U(NUMAX), 
     &     CFX_D(NDMAX), CFY_D(NDMAX), CFZ_D(NDMAX),
     &     CMX_D(NDMAX), CMY_D(NDMAX), CMZ_D(NDMAX),
     &     CFX_G(NGMAX), CFY_G(NGMAX), CFZ_G(NGMAX),
     &     CMX_G(NGMAX), CMY_G(NGMAX), CMZ_G(NGMAX),
     &     CLV_U(NUMAX),
     &     CLV_D(NDMAX),
     &     CLV_G(NGMAX)
C
C
      SINA = SIN(ALFA)
      COSA = COS(ALFA)
C
C***********************************************************************
C...Integrate the forces strip-wise, surface-wise and total-wise
C***********************************************************************
C
C...Calculate strip forces...
C    normalized to strip reference quantities (strip area, chord)
      DO 100 J = 1, NSTRIP
C
        I1  = IJFRST(J)
        NVC = NVSTRP(J)
C
        CR = CHORD(J)
        SR = CHORD(J)*WSTRIP(J)
C
        XTE1 = RLE1(1,J) + CHORD1(J)
        XTE2 = RLE2(1,J) + CHORD2(J)
C
C--- Define local strip lift and drag directions
C--- The "spanwise" vector is cross product of strip normal with X chordline 
        SPN(1) =  0.0
        SPN(2) =  ENSZ(J)
        SPN(3) = -ENSY(J)
C--- Stability axes stream vector defines drag direction
        UDRAG(1) = COSA
        UDRAG(2) = 0.0
        UDRAG(3) = SINA
C--- Lift direction is vector product of "stream" and spanwise vector
        CALL CROSS(UDRAG,SPN,ULIFT)
        ULMAG = SQRT(DOT(ULIFT,ULIFT))
        IF(ULMAG.EQ.0.) THEN
          ULIFT(3) = 1.0
         ELSE
          ULIFT(1) = ULIFT(1)/ULMAG
          ULIFT(2) = ULIFT(2)/ULMAG
          ULIFT(3) = ULIFT(3)/ULMAG
        ENDIF
C
C...Use the strip 1/4 chord location for strip moments
        XR  = RLE(1,J) + 0.25*CR
        YR  = RLE(2,J)
        ZR  = RLE(3,J)
C
        CFX = 0.
        CFY = 0.
        CFZ = 0.
        CMX = 0.
        CMY = 0.
        CMZ = 0.
        CNC(J) = 0.
C
        DO N=1, NUMAX
          CFX_U(N) = 0.
          CFY_U(N) = 0.
          CFZ_U(N) = 0.
          CMX_U(N) = 0.
          CMY_U(N) = 0.
          CMZ_U(N) = 0.
          CNC_U(J,N) = 0.
        ENDDO
C
        DO N=1, NCONTROL
          CFX_D(N) = 0.
          CFY_D(N) = 0.
          CFZ_D(N) = 0.
          CMX_D(N) = 0.
          CMY_D(N) = 0.
          CMZ_D(N) = 0.
          CNC_D(J,N) = 0.
        ENDDO
C
        DO N=1, NDESIGN
          CFX_G(N) = 0.
          CFY_G(N) = 0.
          CFZ_G(N) = 0.
          CMX_G(N) = 0.
          CMY_G(N) = 0.
          CMZ_G(N) = 0.
          CNC_G(J,N) = 0.
        ENDDO
C
C...Sum the forces in the strip as generated by velocity
C    (freestream + rotation + induced) acting on bound vortex 
        DO 40 II = 1, NVC
          I = I1 + (II-1)
C
C------- local moment reference vector from vortex midpoint to strip c/4 pt.
c          R(1) = 0.5*(RV1(1,I) + RV2(1,I)) - XR
c          R(2) = 0.5*(RV1(2,I) + RV2(2,I)) - YR
c          R(3) = 0.5*(RV1(3,I) + RV2(3,I)) - ZR
          R(1) = RV(1,I) - XR
          R(2) = RV(2,I) - YR
          R(3) = RV(3,I) - ZR
C
C------- vector from rotation axes
c          RROT(1) = 0.5*(RV1(1,I) + RV2(1,I)) - XYZREF(1)
c          RROT(2) = 0.5*(RV1(2,I) + RV2(2,I)) - XYZREF(2)
c          RROT(3) = 0.5*(RV1(3,I) + RV2(3,I)) - XYZREF(3)
          RROT(1) = RV(1,I) - XYZREF(1)
          RROT(2) = RV(2,I) - XYZREF(2)
          RROT(3) = RV(3,I) - XYZREF(3)
C
C-------- set total effective velocity = freestream + rotation + induced
          CALL CROSS(RROT,WROT,VROT)
          VEFF(1) = VINF(1) + VROT(1) + WV(1,I)
          VEFF(2) = VINF(2) + VROT(2) + WV(2,I)
          VEFF(3) = VINF(3) + VROT(3) + WV(3,I)
C
C-------- set VEFF sensitivities to freestream,rotation components
          DO K = 1, 3
            VEFF_U(1,K) = WV_U(1,I,K)
            VEFF_U(2,K) = WV_U(2,I,K)
            VEFF_U(3,K) = WV_U(3,I,K)
            VEFF_U(K,K) = 1.0  +  VEFF_U(K,K)
          ENDDO
          DO K = 4, 6
            WROT_U(1) = 0.
            WROT_U(2) = 0.
            WROT_U(3) = 0.
            WROT_U(K-3) = 1.0
            CALL CROSS(RROT,WROT_U,VROT_U)
            VEFF_U(1,K) = VROT_U(1) + WV_U(1,I,K)
            VEFF_U(2,K) = VROT_U(2) + WV_U(2,I,K)
            VEFF_U(3,K) = VROT_U(3) + WV_U(3,I,K)
          ENDDO
C
C-------- Force coefficient on vortex segment is 2(Veff x Gamma)
          G(1) = RV2(1,I)-RV1(1,I)
          G(2) = RV2(2,I)-RV1(2,I)
          G(3) = RV2(3,I)-RV1(3,I)
          CALL CROSS(VEFF, G, F)
          DO N = 1, NUMAX
            CALL CROSS(VEFF_U(1,N), G, F_U(1,N))
          ENDDO
C
          FGAM(1) = 2.0*GAM(I)*F(1)
          FGAM(2) = 2.0*GAM(I)*F(2)
          FGAM(3) = 2.0*GAM(I)*F(3)
          DO N = 1, NUMAX
            FGAM_U(1,N) = 2.0*GAM_U(I,N)*F(1) + 2.0*GAM(I)*F_U(1,N)
            FGAM_U(2,N) = 2.0*GAM_U(I,N)*F(2) + 2.0*GAM(I)*F_U(2,N)
            FGAM_U(3,N) = 2.0*GAM_U(I,N)*F(3) + 2.0*GAM(I)*F_U(3,N)
          ENDDO
          DO N = 1, NCONTROL
            FGAM_D(1,N) = 2.0*GAM_D(I,N)*F(1)
            FGAM_D(2,N) = 2.0*GAM_D(I,N)*F(2)
            FGAM_D(3,N) = 2.0*GAM_D(I,N)*F(3)
          ENDDO
          DO N = 1, NDESIGN
            FGAM_G(1,N) = 2.0*GAM_G(I,N)*F(1)
            FGAM_G(2,N) = 2.0*GAM_G(I,N)*F(2)
            FGAM_G(3,N) = 2.0*GAM_G(I,N)*F(3)
          ENDDO
C
C
C-------- Delta Cp (loading across lifting surface) from vortex 
          FNV = DOT(ENV(1,I),FGAM)
          DCP(I) = FNV / (DXV(I)*WSTRIP(J))
C
          DO N = 1, NUMAX
            FNV_U = DOT(ENV(1,I),FGAM_U(1,N))
            DCP_U(I,N) = FNV_U / (DXV(I)*WSTRIP(J))
          ENDDO
C
          DO N = 1, NCONTROL
            FNV_D = DOT(ENV(1,I),FGAM_D(1,N)) + DOT(ENV_D(1,I,N),FGAM)
            DCP_D(I,N) = FNV_D / (DXV(I)*WSTRIP(J))
          ENDDO
C
          DO N = 1, NDESIGN
            FNV_G = DOT(ENV(1,I),FGAM_G(1,N)) + DOT(ENV_G(1,I,N),FGAM)
            DCP_G(I,N) = FNV_G / (DXV(I)*WSTRIP(J))
          ENDDO
C
C-------- vortex contribution to strip forces
          DCFX = FGAM(1) / SR
          DCFY = FGAM(2) / SR
          DCFZ = FGAM(3) / SR
C
C-------- forces normalized by strip area
          CFX = CFX +  DCFX
          CFY = CFY +  DCFY
          CFZ = CFZ +  DCFZ
C
C-------- moments referred to strip c/4 pt., normalized by strip chord and area
          CMX = CMX + (DCFZ*R(2) - DCFY*R(3))/CR
          CMY = CMY + (DCFX*R(3) - DCFZ*R(1))/CR
          CMZ = CMZ + (DCFY*R(1) - DCFX*R(2))/CR
C
C-------- accumulate strip spanloading = c*CN
          CNC(J) = CNC(J) + CR*(ENSY(J)*DCFY + ENSZ(J)*DCFZ)
C
C-------- freestream and rotation derivatives
          DO N=1, NUMAX
            DCFX_U = FGAM_U(1,N)/SR
            DCFY_U = FGAM_U(2,N)/SR
            DCFZ_U = FGAM_U(3,N)/SR
C
            CFX_U(N) = CFX_U(N) +  DCFX_U
            CFY_U(N) = CFY_U(N) +  DCFY_U
            CFZ_U(N) = CFZ_U(N) +  DCFZ_U
            CMX_U(N) = CMX_U(N) + (DCFZ_U*R(2) - DCFY_U*R(3))/CR
            CMY_U(N) = CMY_U(N) + (DCFX_U*R(3) - DCFZ_U*R(1))/CR
            CMZ_U(N) = CMZ_U(N) + (DCFY_U*R(1) - DCFX_U*R(2))/CR
C
            CNC_U(J,N) = CNC_U(J,N) 
     &                 + CR*(ENSY(J)*DCFY_U + ENSZ(J)*DCFZ_U)
          ENDDO
C
C-------- control derivatives
          DO N=1, NCONTROL
            DCFX_D = FGAM_D(1,N)/SR
            DCFY_D = FGAM_D(2,N)/SR
            DCFZ_D = FGAM_D(3,N)/SR
C
            CFX_D(N) = CFX_D(N) +  DCFX_D
            CFY_D(N) = CFY_D(N) +  DCFY_D
            CFZ_D(N) = CFZ_D(N) +  DCFZ_D
            CMX_D(N) = CMX_D(N) + (DCFZ_D*R(2) - DCFY_D*R(3))/CR
            CMY_D(N) = CMY_D(N) + (DCFX_D*R(3) - DCFZ_D*R(1))/CR
            CMZ_D(N) = CMZ_D(N) + (DCFY_D*R(1) - DCFX_D*R(2))/CR
C
            CNC_D(J,N) = CNC_D(J,N) 
     &                 + CR*(ENSY(J)*DCFY_D + ENSZ(J)*DCFZ_D)
          ENDDO
C
C-------- design derivatives
          DO N=1, NDESIGN
            DCFX_G = FGAM_G(1,N)/SR
            DCFY_G = FGAM_G(2,N)/SR
            DCFZ_G = FGAM_G(3,N)/SR
C
            CFX_G(N) = CFX_G(N) +  DCFX_G
            CFY_G(N) = CFY_G(N) +  DCFY_G
            CFZ_G(N) = CFZ_G(N) +  DCFZ_G
            CMX_G(N) = CMX_G(N) + (DCFZ_G*R(2) - DCFY_G*R(3))/CR
            CMY_G(N) = CMY_G(N) + (DCFX_G*R(3) - DCFZ_G*R(1))/CR
            CMZ_G(N) = CMZ_G(N) + (DCFY_G*R(1) - DCFX_G*R(2))/CR
C
            CNC_G(J,N) = CNC_G(J,N) 
     &                 + CR*(ENSY(J)*DCFY_G + ENSZ(J)*DCFZ_G)
          ENDDO
C
C-------- hinge moments
          DO L = 1, NCONTROL
            RH(1) = RV(1,I) - PHINGE(1,J,L)
            RH(2) = RV(2,I) - PHINGE(2,J,L)
            RH(3) = RV(3,I) - PHINGE(3,J,L)
C
            DFAC = DCONTROL(I,L) / (SREF*CREF)
C
            CALL CROSS(RH,FGAM,MH)
            CHINGE(L) = CHINGE(L) + DOT(MH,VHINGE(1,J,L))*DFAC
C
            DO N = 1, NUMAX
              CALL CROSS(RH,FGAM_U(1,N),MH)
              CHINGE_U(L,N) = CHINGE_U(L,N) + DOT(MH,VHINGE(1,J,L))*DFAC
            ENDDO
            DO N = 1, NCONTROL
              CALL CROSS(RH,FGAM_D(1,N),MH)
              CHINGE_D(L,N) = CHINGE_D(L,N) + DOT(MH,VHINGE(1,J,L))*DFAC
            ENDDO
            DO N = 1, NDESIGN
              CALL CROSS(RH,FGAM_G(1,N),MH)
              CHINGE_G(L,N) = CHINGE_G(L,N) + DOT(MH,VHINGE(1,J,L))*DFAC
            ENDDO
          ENDDO
C
   40   CONTINUE
C
C
        IF(.NOT.LTRFORCE) GO TO 80
C
C...Sum forces in the strip as generated by velocity (freestream + rotation)
C     the parts of trailing legs which lie on the surface
        DO 72 II = 1, NVC
          I = I1 + (II-1)
C
          DO 71 ILEG = 1, 2
            IF(ILEG.EQ.1) THEN
C----------- local moment reference vector from vortex midpoint to strip c/4 pt
             R(1) = 0.5*(RV1(1,I) + XTE1) - XR
             R(2) =      RV1(2,I)         - YR
             R(3) =      RV1(3,I)         - ZR
C 
C----------- vector from rotation axes
             RROT(1) = 0.5*(RV1(1,I) + XTE1) - XYZREF(1)
             RROT(2) =      RV1(2,I)         - XYZREF(2)
             RROT(3) =      RV1(3,I)         - XYZREF(3)
C
C----------- part of trailing leg lying on surface
             G(1) = RV1(1,I) - XTE1
             G(2) = 0.
             G(3) = 0.
C
            ELSE
C----------- local moment reference vector from vortex midpoint to strip c/4 pt
             R(1) = 0.5*(RV2(1,I) + XTE2) - XR
             R(2) =      RV2(2,I)         - YR
             R(3) =      RV2(3,I)         - ZR
C
C----------- vector from rotation axes
             RROT(1) = 0.5*(RV2(1,I) + XTE2) - XYZREF(1)
             RROT(2) =      RV2(2,I)         - XYZREF(2)
             RROT(3) =      RV2(3,I)         - XYZREF(3)
C
C----------- part of trailing leg lying on surface
             G(1) = XTE2 - RV2(1,I)
             G(2) = 0.
             G(3) = 0.
            ENDIF
C
C---------- set total effective velocity = freestream + rotation
            CALL CROSS(RROT,WROT,VROT)
            VEFF(1) = VINF(1) + VROT(1)
            VEFF(2) = VINF(2) + VROT(2)
            VEFF(3) = VINF(3) + VROT(3)
C
C---------- set VEFF sensitivities to freestream,rotation components
            DO K = 1, 3
              VEFF_U(1,K) = 0.
              VEFF_U(2,K) = 0.
              VEFF_U(3,K) = 0.
              VEFF_U(K,K) = 1.0
            ENDDO
            DO K = 4, 6
              WROT_U(1) = 0.
              WROT_U(2) = 0.
              WROT_U(3) = 0.
              WROT_U(K-3) = 1.0
              CALL CROSS(RROT,WROT_U,VROT_U)
              VEFF_U(1,K) = VROT_U(1)
              VEFF_U(2,K) = VROT_U(2)
              VEFF_U(3,K) = VROT_U(3)
            ENDDO
C
C---------- Force coefficient on vortex segment is 2(Veff x Gamma)
            CALL CROSS (VEFF, G, F)
C
            DO N = 1, NUMAX
              CALL CROSS(VEFF_U(1,N), G, F_U(1,N))
            ENDDO
C
            FGAM(1) = 2.0*GAM(I)*F(1)
            FGAM(2) = 2.0*GAM(I)*F(2)
            FGAM(3) = 2.0*GAM(I)*F(3)
            DO N = 1, NUMAX
              FGAM_U(1,N) = 2.0*GAM_U(I,N)*F(1) + 2.0*GAM(I)*F_U(1,N)
              FGAM_U(2,N) = 2.0*GAM_U(I,N)*F(2) + 2.0*GAM(I)*F_U(2,N)
              FGAM_U(3,N) = 2.0*GAM_U(I,N)*F(3) + 2.0*GAM(I)*F_U(3,N)
            ENDDO
            DO N = 1, NCONTROL
              FGAM_D(1,N) = 2.0*GAM_D(I,N)*F(1)
              FGAM_D(2,N) = 2.0*GAM_D(I,N)*F(2)
              FGAM_D(3,N) = 2.0*GAM_D(I,N)*F(3)
            ENDDO
            DO N = 1, NDESIGN
              FGAM_G(1,N) = 2.0*GAM_G(I,N)*F(1)
              FGAM_G(2,N) = 2.0*GAM_G(I,N)*F(2)
              FGAM_G(3,N) = 2.0*GAM_G(I,N)*F(3)
            ENDDO
C
cC---------- Delta Cp (loading across lifting surface) due to vortex 
c            FNV = DOT(ENV(1,I),FGAM)
c            DCP(I) = FNV / (DXV(I)*WSTRIP(J))
cC
c            DO N = 1, NUMAX
c              FNV_U = DOT(ENV(1,I),FGAM_U(1,N))
c              DCP_U(I,N) = FNV_U / (DXV(I)*WSTRIP(J))
c            ENDDO
cC
c            DO N = 1, NCONTROL
c              FNV_D = DOT(ENV(1,I),FGAM_D(1,N)) + DOT(ENV_D(1,I,N),FGAM)
c              DCP_D(I,N) = FNV_D / (DXV(I)*WSTRIP(J))
c            ENDDO
cC
c            DO N = 1, NDESIGN
c              FNV_G = DOT(ENV(1,I),FGAM_G(1,N)) + DOT(ENV_G(1,I,N),FGAM)
c              DCP_G(I,N) = FNV_G / (DXV(I)*WSTRIP(J))
c            ENDDO
C
C
C---------- vortex contribution to strip forces
            DCFX = FGAM(1) / SR
            DCFY = FGAM(2) / SR
            DCFZ = FGAM(3) / SR
C
C---------- forces normalized by strip area
            CFX = CFX +  DCFX
            CFY = CFY +  DCFY
            CFZ = CFZ +  DCFZ
C
C---------- moments referred to strip c/4 pt., normalized by strip chord and area
            CMX = CMX + (DCFZ*R(2) - DCFY*R(3))/CR
            CMY = CMY + (DCFX*R(3) - DCFZ*R(1))/CR
            CMZ = CMZ + (DCFY*R(1) - DCFX*R(2))/CR
C
C---------- accumulate strip spanloading = c*CN
            CNC(J) = CNC(J) + CR*(ENSY(J)*DCFY + ENSZ(J)*DCFZ)
C
C---------- freestream and rotation derivatives
            DO N=1, NUMAX
              DCFX_U = FGAM_U(1,N)/SR
              DCFY_U = FGAM_U(2,N)/SR
              DCFZ_U = FGAM_U(3,N)/SR
C
              CFX_U(N) = CFX_U(N) +  DCFX_U
              CFY_U(N) = CFY_U(N) +  DCFY_U
              CFZ_U(N) = CFZ_U(N) +  DCFZ_U
              CMX_U(N) = CMX_U(N) + (DCFZ_U*R(2) - DCFY_U*R(3))/CR
              CMY_U(N) = CMY_U(N) + (DCFX_U*R(3) - DCFZ_U*R(1))/CR
              CMZ_U(N) = CMZ_U(N) + (DCFY_U*R(1) - DCFX_U*R(2))/CR
C
              CNC_U(J,N) = CNC_U(J,N) 
     &                   + CR*(ENSY(J)*DCFY_U + ENSZ(J)*DCFZ_U)
            ENDDO
C
C---------- control derivatives
            DO N=1, NCONTROL
              DCFX_D = FGAM_D(1,N)/SR
              DCFY_D = FGAM_D(2,N)/SR
              DCFZ_D = FGAM_D(3,N)/SR
C  
              CFX_D(N) = CFX_D(N) +  DCFX_D
              CFY_D(N) = CFY_D(N) +  DCFY_D
              CFZ_D(N) = CFZ_D(N) +  DCFZ_D
              CMX_D(N) = CMX_D(N) + (DCFZ_D*R(2) - DCFY_D*R(3))/CR
              CMY_D(N) = CMY_D(N) + (DCFX_D*R(3) - DCFZ_D*R(1))/CR
              CMZ_D(N) = CMZ_D(N) + (DCFY_D*R(1) - DCFX_D*R(2))/CR
C  
              CNC_D(J,N) = CNC_D(J,N) 
     &                   + CR*(ENSY(J)*DCFY_D + ENSZ(J)*DCFZ_D)
            ENDDO
C
C---------- design derivatives
            DO N=1, NDESIGN
              DCFX_G = FGAM_G(1,N)/SR
              DCFY_G = FGAM_G(2,N)/SR
              DCFZ_G = FGAM_G(3,N)/SR
C
              CFX_G(N) = CFX_G(N) +  DCFX_G
              CFY_G(N) = CFY_G(N) +  DCFY_G
              CFZ_G(N) = CFZ_G(N) +  DCFZ_G
              CMX_G(N) = CMX_G(N) + (DCFZ_G*R(2) - DCFY_G*R(3))/CR
              CMY_G(N) = CMY_G(N) + (DCFX_G*R(3) - DCFZ_G*R(1))/CR
              CMZ_G(N) = CMZ_G(N) + (DCFY_G*R(1) - DCFX_G*R(2))/CR
C
              CNC_G(J,N) = CNC_G(J,N) 
     &                   + CR*(ENSY(J)*DCFY_G + ENSZ(J)*DCFZ_G)
            ENDDO
C
cC---------- hinge moments
c            DO L=1, NCONTROL
c              RH(1) = RV(1,I) - PHINGE(1,J,L)
c              RH(2) = RV(2,I) - PHINGE(2,J,L)
c              RH(3) = RV(3,I) - PHINGE(3,J,L)
cC
c              DFAC = DCONTROL(I,L) / (SREF * CREF)
cC
c              CALL CROSS(RH,FGAM,MH)
c              CHINGE(L) = CHINGE(L) + DOT(MH,VHINGE(1,J,L))*DFAC
cC
c              DO N = 1, NUMAX
c                CALL CROSS(RH,FGAM_U(1,N),MH)
c                CHINGE_U(L,N) = CHINGE_U(L,N) + DOT(MH,VHINGE(1,J,L))*DFAC
c              ENDDO
c              DO N = 1, NCONTROL
c                CALL CROSS(RH,FGAM_D(1,N),MH)
c                CHINGE_D(L,N) = CHINGE_D(L,N) + DOT(MH,VHINGE(1,J,L))*DFAC
c              ENDDO
c              DO N = 1, NDESIGN
c                CALL CROSS(RH,FGAM_G(1,N),MH)
c                CHINGE_G(L,N) = CHINGE_G(L,N) + DOT(MH,VHINGE(1,J,L))*DFAC
c              ENDDO
c            ENDDO
C
   71     CONTINUE
   72   CONTINUE
 80     CONTINUE
C
C
C*******************************************************************
C--- Drag terms due to viscous effects
C    Drag forces are assumed to be characterized by velocity at the c/4 
C    point and are assumed to act thru the same point. CD is defined by 
C    user-specified CD(CL) polar.  Drag comes from function lookup on 
C    section polar drag using local lift coefficient.  
C
        CDV_LSTRP(J) = 0.0
C
        IF(LVISC.AND.LVISCSTRP(J)) THEN
C--- local moment reference vector from ref point to c/4 point
c         R(1) = XR - XR
c         R(2) = YR - YR
c         R(3) = ZR - ZR
C--- Get rotational velocity at strip 1/4 chord reference point 
         RROT(1) = XR - XYZREF(1)
         RROT(2) = YR - XYZREF(2)
         RROT(3) = ZR - XYZREF(3)
C--- Onset velocity at strip c/4 = freestream + rotation
         CALL CROSS(RROT,WROT,VROT)
         VEFF(1) = VINF(1) + VROT(1)
         VEFF(2) = VINF(2) + VROT(2)
         VEFF(3) = VINF(3) + VROT(3)
         VEFFMAG = SQRT(VEFF(1)**2 +VEFF(2)**2 +VEFF(3)**2)
C
C------- set sensitivities to freestream,rotation components
         DO K = 1, 3
           VEFF_U(1,K) = 0.
           VEFF_U(2,K) = 0.
           VEFF_U(3,K) = 0.
         ENDDO
         VEFF_U(1,1) = 1.0
         VEFF_U(2,2) = 1.0
         VEFF_U(3,3) = 1.0
         DO K = 4, 6
           WROT_U(1) = 0.
           WROT_U(2) = 0.
           WROT_U(3) = 0.
           WROT_U(K-3) = 1.0
           CALL CROSS(RROT,WROT_U,VROT_U)
           VEFF_U(1,K) = VROT_U(1)
           VEFF_U(2,K) = VROT_U(2)
           VEFF_U(3,K) = VROT_U(3)
         ENDDO
C
C--- Generate CD from stored function using strip CL as parameter
         CLV = ULIFT(1)*CFX + ULIFT(2)*CFY + ULIFT(3)*CFZ
         DO N = 1, NUMAX
           CLV_U(N) = ENSY(J)* CFY_U(N)
     &              + ENSZ(J)*(CFZ_U(N)*COSA - CFX_U(N)*SINA)  
         END DO
C
         DO N = 1, NCONTROL
           CLV_D(N) = ENSY(J)* CFY_D(N)
     &              + ENSZ(J)*(CFZ_D(N)*COSA - CFX_D(N)*SINA)  
         END DO
C
         DO N = 1, NDESIGN
           CLV_G(N) = ENSY(J)* CFY_G(N)
     &              + ENSZ(J)*(CFZ_G(N)*COSA - CFX_G(N)*SINA)  
         END DO
C
         CALL CDCL(J,CLV,CDV,CDV_CLV)
C
C--- Strip viscous force contribution (per unit strip area)
         DCVFX = VEFF(1)*VEFFMAG * CDV
         DCVFY = VEFF(2)*VEFFMAG * CDV
         DCVFZ = VEFF(3)*VEFFMAG * CDV
C
C--- Add viscous terms to strip forces and moments
         CFX = CFX +  DCVFX
         CFY = CFY +  DCVFY
         CFZ = CFZ +  DCVFZ
C--- Viscous forces acting at c/4 have no effect on moments at c/4 pt.
c         CMX = CMX + (DCVFZ*R(2) - DCVFY*R(3))/CR
c         CMY = CMY + (DCVFX*R(3) - DCVFZ*R(1))/CR
c         CMZ = CMZ + (DCVFY*R(1) - DCVFX*R(2))/CR
C
         CDV_LSTRP(J) = UDRAG(1)*DCVFX + UDRAG(2)*DCVFY + UDRAG(3)*DCVFZ
C
C--- Add the sensitivity of viscous forces to the flow conditions
         DO N=1, NUMAX
           DCVFX_U = (VEFF_U(1,N)*(VEFFMAG + VEFF(1)**2/VEFFMAG))*CDV
     &             +  VEFF(1)    * VEFFMAG * CDV_CLV*CLV_U(N)
           DCVFY_U = (VEFF_U(2,N)*(VEFFMAG + VEFF(2)**2/VEFFMAG))*CDV
     &             +  VEFF(2)    * VEFFMAG * CDV_CLV*CLV_U(N)
           DCVFZ_U = (VEFF_U(3,N)*(VEFFMAG + VEFF(3)**2/VEFFMAG))*CDV
     &             +  VEFF(3)    * VEFFMAG * CDV_CLV*CLV_U(N)
C
           CFX_U(N) = CFX_U(N) +  DCVFX_U
           CFY_U(N) = CFY_U(N) +  DCVFY_U
           CFZ_U(N) = CFZ_U(N) +  DCVFZ_U
C--- Viscous forces acting at c/4 have no effect on moments at c/4 pt.
c           CMX_U(N) = CMX_U(N) + (DCVFZ_U*R(2) - DCVFY_U*R(3))/CR
c           CMY_U(N) = CMY_U(N) + (DCVFX_U*R(3) - DCVFZ_U*R(1))/CR
c           CMZ_U(N) = CMZ_U(N) + (DCVFY_U*R(1) - DCVFX_U*R(2))/CR
C
           CNC_U(J,N) = CNC_U(J,N) 
     &                + CR * (ENSY(J)*DCVFY_U + ENSZ(J)*DCVFZ_U)
         ENDDO
C
         DO N=1, NCONTROL
           DCVFX_D = VEFF(1)*VEFFMAG * CDV_CLV*CLV_D(N)
           DCVFY_D = VEFF(2)*VEFFMAG * CDV_CLV*CLV_D(N)
           DCVFZ_D = VEFF(3)*VEFFMAG * CDV_CLV*CLV_D(N)
C
           CFX_D(N) = CFX_D(N) +  DCVFX_D
           CFY_D(N) = CFY_D(N) +  DCVFY_D
           CFZ_D(N) = CFZ_D(N) +  DCVFZ_D
C--- Viscous forces acting at c/4 have no effect on moments at c/4 pt.
c           CMX_D(N) = CMX_D(N) + (DCVFZ_D*R(2) - DCVFY_D*R(3))/CR
c           CMY_D(N) = CMY_D(N) + (DCVFX_D*R(3) - DCVFZ_D*R(1))/CR
c           CMZ_D(N) = CMZ_D(N) + (DCVFY_D*R(1) - DCVFX_D*R(2))/CR
C
           CNC_D(J,N) = CNC_D(J,N) 
     &                + CR * (ENSY(J)*DCVFY_D + ENSZ(J)*DCVFZ_D)
         ENDDO
C
         DO N=1, NDESIGN
           DCVFX_G = VEFF(1)*VEFFMAG * CDV_CLV*CLV_G(N)
           DCVFY_G = VEFF(2)*VEFFMAG * CDV_CLV*CLV_G(N)
           DCVFZ_G = VEFF(3)*VEFFMAG * CDV_CLV*CLV_G(N)
C
           CFX_G(N) = CFX_G(N) +  DCVFX_G
           CFY_G(N) = CFY_G(N) +  DCVFY_G
           CFZ_G(N) = CFZ_G(N) +  DCVFZ_G
C--- Viscous forces acting at c/4 have no effect on moments at c/4 pt.
c           CMX_G(N) = CMX_G(N) + (DCVFZ_G*R(2) - DCVFY_G*R(3))/CR
c           CMY_G(N) = CMY_G(N) + (DCVFX_G*R(3) - DCVFZ_G*R(1))/CR
c           CMZ_G(N) = CMZ_G(N) + (DCVFY_G*R(1) - DCVFX_G*R(2))/CR
C
           CNC_G(J,N) = CNC_G(J,N) 
     &                + CR * (ENSY(J)*DCVFY_G + ENSZ(J)*DCVFZ_G)
         ENDDO
C
        ENDIF        
C
C*******************************************************************
C
C...Store strip X,Y,Z body axes forces 
C   (these are normalized by strip area and moments are referred to
C    c/4 point and are normalized by strip chord and area)
        CF_STRP(1,J) = CFX
        CF_STRP(2,J) = CFY
        CF_STRP(3,J) = CFZ
        CM_STRP(1,J) = CMX
        CM_STRP(2,J) = CMY
        CM_STRP(3,J) = CMZ
C
C...Transform strip body axes forces into stability axes
        CDSTRP(J) =  CFX*COSA + CFZ*SINA
        CLSTRP(J) = -CFX*SINA + CFZ*COSA 
        CXSTRP(J) =  CFX
        CYSTRP(J) =  CFY
        CZSTRP(J) =  CFZ
C
        CDST_A(J) = -CFX*SINA + CFZ*COSA
        CLST_A(J) = -CFX*COSA - CFZ*SINA 
C
        DO N=1, NUMAX
          CDST_U(J,N) =  CFX_U(N)*COSA + CFZ_U(N)*SINA
          CLST_U(J,N) = -CFX_U(N)*SINA + CFZ_U(N)*COSA 
          CXST_U(J,N) =  CFX_U(N)
          CYST_U(J,N) =  CFY_U(N)
          CZST_U(J,N) =  CFZ_U(N)
        END DO
C
        DO N=1, NCONTROL
          CDST_D(J,N) =  CFX_D(N)*COSA + CFZ_D(N)*SINA
          CLST_D(J,N) = -CFX_D(N)*SINA + CFZ_D(N)*COSA 
          CXST_D(J,N) =  CFX_D(N)
          CYST_D(J,N) =  CFY_D(N)
          CZST_D(J,N) =  CFZ_D(N)
        END DO
C
        DO N=1, NDESIGN
          CDST_G(J,N) =  CFX_G(N)*COSA + CFZ_G(N)*SINA
          CLST_G(J,N) = -CFX_G(N)*SINA + CFZ_G(N)*COSA 
          CXST_G(J,N) =  CFX_G(N)
          CYST_G(J,N) =  CFY_G(N)
          CZST_G(J,N) =  CFZ_G(N)
        END DO
C
C... Set strip moments about the overall moment reference point XYZREF 
C     (still normalized by strip area and chord)
        R(1) = XR - XYZREF(1)
        R(2) = YR - XYZREF(2)
        R(3) = ZR - XYZREF(3)
        CRSTRP(J) = CMX + (CFZ*R(2) - CFY*R(3))/CR
        CMSTRP(J) = CMY + (CFX*R(3) - CFZ*R(1))/CR
        CNSTRP(J) = CMZ + (CFY*R(1) - CFX*R(2))/CR
C
        DO N=1, NUMAX
          CRST_U(J,N) = CMX_U(N) + (CFZ_U(N)*R(2) - CFY_U(N)*R(3))/CR
          CMST_U(J,N) = CMY_U(N) + (CFX_U(N)*R(3) - CFZ_U(N)*R(1))/CR
          CNST_U(J,N) = CMZ_U(N) + (CFY_U(N)*R(1) - CFX_U(N)*R(2))/CR
        ENDDO
C
        DO N=1, NCONTROL
          CRST_D(J,N) = CMX_D(N) + (CFZ_D(N)*R(2) - CFY_D(N)*R(3))/CR
          CMST_D(J,N) = CMY_D(N) + (CFX_D(N)*R(3) - CFZ_D(N)*R(1))/CR
          CNST_D(J,N) = CMZ_D(N) + (CFY_D(N)*R(1) - CFX_D(N)*R(2))/CR
        ENDDO
C
        DO N=1, NDESIGN
          CRST_G(J,N) = CMX_G(N) + (CFZ_G(N)*R(2) - CFY_G(N)*R(3))/CR
          CMST_G(J,N) = CMY_G(N) + (CFX_G(N)*R(3) - CFZ_G(N)*R(1))/CR
          CNST_G(J,N) = CMZ_G(N) + (CFY_G(N)*R(1) - CFX_G(N)*R(2))/CR
        ENDDO
C
C...Take components of X,Y,Z forces in local strip axes 
C   (axial/normal and lift/drag)
C    in plane normal to (possibly dihedralled) strip
        CL_LSTRP(J) = ULIFT(1)*CFX + ULIFT(2)*CFY + ULIFT(3)*CFZ
        CD_LSTRP(J) = UDRAG(1)*CFX + UDRAG(2)*CFY + UDRAG(3)*CFZ
        CAXLSTRP(J) = CFX
        CNRMSTRP(J) = ENSY(J)*CFY + ENSZ(J)*CFZ
        CMC4(J)     = ENSZ(J)*CMY - ENSY(J)*CMZ
C
C------ vector at chord reference point from rotation axes
        RROT(1) = XSREF(J) - XYZREF(1)
        RROT(2) = YSREF(J) - XYZREF(2)
        RROT(3) = ZSREF(J) - XYZREF(3)
C
C------ set total effective velocity = freestream + rotation
        CALL CROSS(RROT,WROT,VROT)
        VEFF(1) = VINF(1) + VROT(1)
        VEFF(2) = VINF(2) + VROT(2)
        VEFF(3) = VINF(3) + VROT(3)
C
        VSQ = VEFF(1)**2 + VEFF(2)**2 + VEFF(3)**2
        IF(VSQ .EQ. 0.0) THEN
         VSQI = 1.0
        ELSE
         VSQI = 1.0 / VSQ
        ENDIF
C
C------ spanwise and perpendicular velocity components
        VSPAN = VEFF(1)*ESS(1,J) + VEFF(2)*ESS(2,J) + VEFF(3)*ESS(3,J)
        VPERP(1) = VEFF(1) - ESS(1,J)*VSPAN
        VPERP(2) = VEFF(2) - ESS(2,J)*VSPAN
        VPERP(3) = VEFF(3) - ESS(3,J)*VSPAN
C
        VPSQ = VPERP(1)**2 + VPERP(2)**2 + VPERP(3)**2
        IF(VPSQ .EQ. 0.0) THEN
         VPSQI = 1.0
        ELSE
         VPSQI = 1.0 / VPSQ
        ENDIF
ccc     CLTSTRP(J) = CNRMSTRP(J) * VPSQI
        CLTSTRP(J) = CL_LSTRP(J) * VPSQI
        CLASTRP(J) = CL_LSTRP(J) * VSQI
C
C--- Moment about strip LE midpoint in direction of LE segment
        R(1) = XR - RLE(1,J)
        R(2) = YR - RLE(2,J)
        R(3) = ZR - RLE(3,J)
        DELX = RLE2(1,J) - RLE1(1,J)
        DELY = RLE2(2,J) - RLE1(2,J)
        DELZ = RLE2(3,J) - RLE1(3,J)
C
        IF(IMAGS(NSURFS(J)).LT.0) THEN 
         DELX = -DELX
         DELY = -DELY
         DELZ = -DELZ
        ENDIF
        DMAG = SQRT(DELX**2+DELY**2+DELZ**2)
        CMLE(J) = 0.0
        IF(DMAG.NE.0.0) THEN
         CMLE(J) = DELX/DMAG*(CMX + (CFZ*R(2) - CFY*R(3))/CR)
     &           + DELY/DMAG*(CMY + (CFX*R(3) - CFZ*R(1))/CR)
     &           + DELZ/DMAG*(CMZ + (CFY*R(1) - CFX*R(2))/CR)
        ENDIF
C
  100 CONTINUE
C
C
C
C...Surface forces and moments summed from strip forces...
C   XXSURF values normalized to configuration reference quantities
C   XX_SRF values normalized to each surface reference quantities
      DO 150 IS = 1, NSURF
        CDSURF(IS) = 0.
        CLSURF(IS) = 0.
        CXSURF(IS) = 0.
        CYSURF(IS) = 0.
        CZSURF(IS) = 0.
        CRSURF(IS) = 0.
        CMSURF(IS) = 0.
        CNSURF(IS) = 0.
        CDVSURF(IS) = 0.
C
        CDS_A(IS) = 0.
        CLS_A(IS) = 0.
        DO N=1, NUMAX
          CDS_U(IS,N) = 0.
          CLS_U(IS,N) = 0.
          CXS_U(IS,N) = 0.
          CYS_U(IS,N) = 0.
          CZS_U(IS,N) = 0.
          CRS_U(IS,N) = 0.
          CMS_U(IS,N) = 0.
          CNS_U(IS,N) = 0.
        ENDDO
        DO N=1, NCONTROL
          CDS_D(IS,N) = 0.
          CLS_D(IS,N) = 0.
          CXS_D(IS,N) = 0.
          CYS_D(IS,N) = 0.
          CZS_D(IS,N) = 0.
          CRS_D(IS,N) = 0.
          CMS_D(IS,N) = 0.
          CNS_D(IS,N) = 0.
        ENDDO
        DO N=1, NDESIGN
          CDS_G(IS,N) = 0.
          CLS_G(IS,N) = 0.
          CXS_G(IS,N) = 0.
          CYS_G(IS,N) = 0.
          CZS_G(IS,N) = 0.
          CRS_G(IS,N) = 0.
          CMS_G(IS,N) = 0.
          CNS_G(IS,N) = 0.
        ENDDO
C
C--- Surface body axes forces and moments
        DO L = 1, 3
          CF_SRF(L,IS) = 0.0
          CM_SRF(L,IS) = 0.0
          ENAVE(L)    = 0.0
        END DO
C
        NSTRPS = NJ(IS)
        DO 120 JJ = 1, NSTRPS
          J = JFRST(IS) + JJ-1
          SR = CHORD(J)*WSTRIP(J)
          CR = CHORD(J)
          XR = RLE(1,J) + 0.25*CHORD(J)
          YR = RLE(2,J)
          ZR = RLE(3,J)
C
          ENAVE(1) = 0.0
          ENAVE(2) = ENAVE(2) + SR*ENSY(J)
          ENAVE(3) = ENAVE(3) + SR*ENSZ(J)
C
          CDSURF(IS) = CDSURF(IS) + CDSTRP(J)*SR/SREF
          CLSURF(IS) = CLSURF(IS) + CLSTRP(J)*SR/SREF
C
          CXSURF(IS) = CXSURF(IS) + CXSTRP(J)*SR/SREF
          CYSURF(IS) = CYSURF(IS) + CYSTRP(J)*SR/SREF
          CZSURF(IS) = CZSURF(IS) + CZSTRP(J)*SR/SREF
C
          CRSURF(IS) = CRSURF(IS) + CRSTRP(J)*(SR/SREF)*(CR/BREF)
          CMSURF(IS) = CMSURF(IS) + CMSTRP(J)*(SR/SREF)*(CR/CREF)
          CNSURF(IS) = CNSURF(IS) + CNSTRP(J)*(SR/SREF)*(CR/BREF)
C
C--- Bug fix, HHY/S.Allmaras 
          CDVSURF(IS)  = CDVSURF(IS) + CDV_LSTRP(J)*(SR/SREF)
C
          CDS_A(IS) = CDS_A(IS) + CDST_A(J)*SR/SREF
          CLS_A(IS) = CLS_A(IS) + CLST_A(J)*SR/SREF
C
          DO N=1, NUMAX
            CDS_U(IS,N) = CDS_U(IS,N) + CDST_U(J,N)*SR/SREF
            CLS_U(IS,N) = CLS_U(IS,N) + CLST_U(J,N)*SR/SREF
C
            CXS_U(IS,N) = CXS_U(IS,N) + CXST_U(J,N)*SR/SREF
            CYS_U(IS,N) = CYS_U(IS,N) + CYST_U(J,N)*SR/SREF
            CZS_U(IS,N) = CZS_U(IS,N) + CZST_U(J,N)*SR/SREF
C
            CRS_U(IS,N) = CRS_U(IS,N) + CRST_U(J,N)*(SR/SREF)*(CR/BREF)
            CMS_U(IS,N) = CMS_U(IS,N) + CMST_U(J,N)*(SR/SREF)*(CR/CREF)
            CNS_U(IS,N) = CNS_U(IS,N) + CNST_U(J,N)*(SR/SREF)*(CR/BREF)
          ENDDO
C
          DO N=1, NCONTROL
            CDS_D(IS,N) = CDS_D(IS,N) + CDST_D(J,N)*SR/SREF
            CLS_D(IS,N) = CLS_D(IS,N) + CLST_D(J,N)*SR/SREF
C
            CXS_D(IS,N) = CXS_D(IS,N) + CXST_D(J,N)*SR/SREF
            CYS_D(IS,N) = CYS_D(IS,N) + CYST_D(J,N)*SR/SREF
            CZS_D(IS,N) = CZS_D(IS,N) + CZST_D(J,N)*SR/SREF
C
            CRS_D(IS,N) = CRS_D(IS,N) + CRST_D(J,N)*(SR/SREF)*(CR/BREF)
            CMS_D(IS,N) = CMS_D(IS,N) + CMST_D(J,N)*(SR/SREF)*(CR/CREF)
            CNS_D(IS,N) = CNS_D(IS,N) + CNST_D(J,N)*(SR/SREF)*(CR/BREF)
          ENDDO
C
          DO N=1, NDESIGN
            CDS_G(IS,N) = CDS_G(IS,N) + CDST_G(J,N)*SR/SREF
            CLS_G(IS,N) = CLS_G(IS,N) + CLST_G(J,N)*SR/SREF
C
            CXS_G(IS,N) = CXS_G(IS,N) + CXST_G(J,N)*SR/SREF
            CYS_G(IS,N) = CYS_G(IS,N) + CYST_G(J,N)*SR/SREF
            CZS_G(IS,N) = CZS_G(IS,N) + CZST_G(J,N)*SR/SREF
C
            CRS_G(IS,N) = CRS_G(IS,N) + CRST_G(J,N)*(SR/SREF)*(CR/BREF)
            CMS_G(IS,N) = CMS_G(IS,N) + CMST_G(J,N)*(SR/SREF)*(CR/CREF)
            CNS_G(IS,N) = CNS_G(IS,N) + CNST_G(J,N)*(SR/SREF)*(CR/BREF)
          ENDDO
C
C--- reference point for surface LE (hinge) moments
C    defined by surface hinge vector direction thru first strip LE point
          IF(IMAGS(IS).GE.0) THEN
            R(1) = XR - RLE1(1,JFRST(IS))
            R(2) = YR - RLE1(2,JFRST(IS))
            R(3) = ZR - RLE1(3,JFRST(IS))
           ELSE
            R(1) = XR - RLE2(1,JFRST(IS))
            R(2) = YR - RLE2(2,JFRST(IS))
            R(3) = ZR - RLE2(3,JFRST(IS))
          ENDIF
C--- Surface forces and moments (about LE ref point, normalized locally) 
          DO L = 1, 3
            L1 = MOD(L, 3) + 1
            L2 = MOD(L1,3) + 1
C
            CF_SRF(L,IS) = CF_SRF(L,IS) + CF_STRP(L,J)*SR/SSURF(IS)
C
            DCM = SR/SSURF(IS) * CR/CAVESURF(IS) *
     &          ( CM_STRP(L,J) 
     &          + CF_STRP(L2,J)*R(L1)/CR 
     &          - CF_STRP(L1,J)*R(L2)/CR )
C
            CM_SRF(L,IS) = CM_SRF(L,IS) + DCM 
          END DO
C
  120   CONTINUE
C
C--- To define surface CL and CD we need local lift and drag directions...
C--- Define drag and lift directions for surface using average strip normal
        ENAVE(1) = ENAVE(1)/SSURF(IS)
        ENAVE(2) = ENAVE(2)/SSURF(IS)
        ENAVE(3) = ENAVE(3)/SSURF(IS)
        ENMAG = SQRT(DOT(ENAVE,ENAVE))
        IF(ENMAG.EQ.0.) THEN
          ENAVE(3) = 1.0
         ELSE
          ENAVE(1) = ENAVE(1)/ENMAG 
          ENAVE(2) = ENAVE(2)/ENMAG 
          ENAVE(3) = ENAVE(3)/ENMAG 
        ENDIF
C--- Define a "spanwise" vector with cross product of average surface normal 
C    with chordline (x direction)
        SPN(1) =  0.0
        SPN(2) =  ENAVE(3)
        SPN(3) = -ENAVE(2)
C--- Stability axes stream vector defines drag direction
        UDRAG(1) = COSA
        UDRAG(2) = 0.0
        UDRAG(3) = SINA
C--- Lift direction is vector product of "stream" and spanwise vector
        CALL CROSS(UDRAG,SPN,ULIFT)
        ULMAG = SQRT(DOT(ULIFT,ULIFT))
        IF(ULMAG.EQ.0.) THEN
          ULIFT(3) = 1.0
         ELSE
          ULIFT(1) = ULIFT(1)/ULMAG
          ULIFT(2) = ULIFT(2)/ULMAG
          ULIFT(3) = ULIFT(3)/ULMAG
        ENDIF
        CL_SRF(IS) = DOT(ULIFT,CF_SRF(1,IS))
        CD_SRF(IS) = DOT(UDRAG,CF_SRF(1,IS))
C--- Surface hinge moments defined by surface LE moment about hinge vector 
ccc        CMLE_SRF(IS) = DOT(CM_SRF(1,IS),VHINGE(1,IS))
C
C
C-------------------------------------------------
        IF(LFLOAD(IS)) THEN
C------- Total forces summed from surface forces...
C-         normalized to configuration reference quantities
         CDTOT = CDTOT + CDSURF(IS)
         CLTOT = CLTOT + CLSURF(IS)
         CXTOT = CXTOT + CXSURF(IS)
         CYTOT = CYTOT + CYSURF(IS)
         CZTOT = CZTOT + CZSURF(IS)
         CRTOT = CRTOT + CRSURF(IS)
         CMTOT = CMTOT + CMSURF(IS)
         CNTOT = CNTOT + CNSURF(IS)
         CDVTOT = CDVTOT + CDVSURF(IS)
C
         CDTOT_A = CDTOT_A + CDS_A(IS)
         CLTOT_A = CLTOT_A + CLS_A(IS)
C
         DO N=1, NUMAX
           CDTOT_U(N) = CDTOT_U(N) + CDS_U(IS,N)
           CLTOT_U(N) = CLTOT_U(N) + CLS_U(IS,N)
           CXTOT_U(N) = CXTOT_U(N) + CXS_U(IS,N)
           CYTOT_U(N) = CYTOT_U(N) + CYS_U(IS,N)
           CZTOT_U(N) = CZTOT_U(N) + CZS_U(IS,N)
           CRTOT_U(N) = CRTOT_U(N) + CRS_U(IS,N)
           CMTOT_U(N) = CMTOT_U(N) + CMS_U(IS,N)
           CNTOT_U(N) = CNTOT_U(N) + CNS_U(IS,N)
         ENDDO
C
         DO N=1, NCONTROL
           CDTOT_D(N) = CDTOT_D(N) + CDS_D(IS,N)
           CLTOT_D(N) = CLTOT_D(N) + CLS_D(IS,N)
           CXTOT_D(N) = CXTOT_D(N) + CXS_D(IS,N)
           CYTOT_D(N) = CYTOT_D(N) + CYS_D(IS,N)
           CZTOT_D(N) = CZTOT_D(N) + CZS_D(IS,N)
           CRTOT_D(N) = CRTOT_D(N) + CRS_D(IS,N)
           CMTOT_D(N) = CMTOT_D(N) + CMS_D(IS,N)
           CNTOT_D(N) = CNTOT_D(N) + CNS_D(IS,N)
         ENDDO
C
         DO N=1, NDESIGN
           CDTOT_G(N) = CDTOT_G(N) + CDS_G(IS,N)
           CLTOT_G(N) = CLTOT_G(N) + CLS_G(IS,N)
           CXTOT_G(N) = CXTOT_G(N) + CXS_G(IS,N)
           CYTOT_G(N) = CYTOT_G(N) + CYS_G(IS,N)
           CZTOT_G(N) = CZTOT_G(N) + CZS_G(IS,N)
           CRTOT_G(N) = CRTOT_G(N) + CRS_G(IS,N)
           CMTOT_G(N) = CMTOT_G(N) + CMS_G(IS,N)
           CNTOT_G(N) = CNTOT_G(N) + CNS_G(IS,N)
         ENDDO
        ENDIF
C-------------------------------------------------
C
  150 CONTINUE
C
C--- If case is XZ symmetric (IYSYM=1), add contributions from images,
C    zero out the asymmetric forces and double the symmetric ones
      IF(IYSYM.EQ.1) THEN
        CDTOT   = 2.0 * CDTOT
        CLTOT   = 2.0 * CLTOT
        CXTOT   = 2.0 * CXTOT
        CYTOT   = 0.
        CZTOT   = 2.0 * CZTOT
        CRTOT   = 0.
        CMTOT   = 2.0 * CMTOT
        CNTOT   = 0.
        CDVTOT  = 2.0 * CDVTOT
C
        CDTOT_A = 2.0 * CDTOT_A
        CLTOT_A = 2.0 * CLTOT_A
C
        DO N=1, NUMAX
          CDTOT_U(N) = 2.0 * CDTOT_U(N)
          CLTOT_U(N) = 2.0 * CLTOT_U(N)
          CXTOT_U(N) = 2.0 * CXTOT_U(N)
          CYTOT_U(N) = 0.
          CZTOT_U(N) = 2.0 * CZTOT_U(N)
          CRTOT_U(N) = 0.
          CMTOT_U(N) = 2.0 * CMTOT_U(N)
          CNTOT_U(N) = 0.
        ENDDO
C
        DO N=1, NCONTROL
          CDTOT_D(N) = 2.0 * CDTOT_D(N)
          CLTOT_D(N) = 2.0 * CLTOT_D(N)
          CXTOT_D(N) = 2.0 * CXTOT_D(N)
          CYTOT_D(N) = 0.
          CZTOT_D(N) = 2.0 * CZTOT_D(N)
          CRTOT_D(N) = 0.
          CMTOT_D(N) = 2.0 * CMTOT_D(N)
          CNTOT_D(N) = 0.
        ENDDO
C
        DO N=1, NDESIGN
          CDTOT_G(N) = 2.0 * CDTOT_G(N)
          CLTOT_G(N) = 2.0 * CLTOT_G(N)
          CXTOT_G(N) = 2.0 * CXTOT_G(N)
          CYTOT_G(N) = 0.
          CZTOT_G(N) = 2.0 * CZTOT_G(N)
          CRTOT_G(N) = 0.
          CMTOT_G(N) = 2.0 * CMTOT_G(N)
          CNTOT_G(N) = 0.
        ENDDO
      ENDIF
C
      RETURN
      END ! SFFORC
C


      SUBROUTINE BDFORC
      INCLUDE 'AVL.INC'
C
      REAL RROT(3)
      REAL VEFF(3)    , VROT(3)  ,
     &     VEFF_U(3,6), VROT_U(3), WROT_U(3)
      REAL DRL(3), ESL(3), 
     &     FB(3), FB_U(3,NUMAX),
     &     MB(3), MB_U(3,NUMAX)
      REAL CDBDY_U(NUMAX), CLBDY_U(NUMAX), 
     &     CXBDY_U(NUMAX), CYBDY_U(NUMAX),  CZBDY_U(NUMAX), 
     &     CRBDY_U(NUMAX), CMBDY_U(NUMAX),  CNBDY_U(NUMAX)
C
C
      BETM = SQRT(1.0 - MACH**2)
C
      SINA = SIN(ALFA)
      COSA = COS(ALFA)
C
C
C---- add on body force contributions
      DO 200 IB = 1, NBODY
        CDBDY(IB) = 0.0
        CLBDY(IB) = 0.0
        CXBDY(IB) = 0.0
        CYBDY(IB) = 0.0
        CZBDY(IB) = 0.0
        CRBDY(IB) = 0.0
        CMBDY(IB) = 0.0
        CNBDY(IB) = 0.0
C
        DO IU = 1, 6
          CDBDY_U(IU) = 0.0
          CLBDY_U(IU) = 0.0
          CXBDY_U(IU) = 0.0
          CYBDY_U(IU) = 0.0
          CZBDY_U(IU) = 0.0
          CRBDY_U(IU) = 0.0
          CMBDY_U(IU) = 0.0
          CNBDY_U(IU) = 0.0
        ENDDO
C
        DO 205 ILSEG = 1, NL(IB)-1
          L1 = LFRST(IB) + ILSEG - 1
          L2 = LFRST(IB) + ILSEG
C
          L = L1
C
          DRL(1) = (RL(1,L2) - RL(1,L1))/BETM
          DRL(2) =  RL(2,L2) - RL(2,L1)
          DRL(3) =  RL(3,L2) - RL(3,L1)
          DRLMAG = SQRT(DRL(1)**2 + DRL(2)**2 + DRL(3)**2)
          IF(DRLMAG.EQ.0.0) THEN
           DRLMI = 0.0
          ELSE
           DRLMI = 1.0/DRLMAG
          ENDIF
C
          DIA = RADL(L1) + RADL(L2)
          IF(DIA.LE.0.0) THEN
           DINV = 0.0
          ELSE
           DINV = 1.0/DIA
          ENDIF
C
C-------- unit vector along line segment
          ESL(1) = DRL(1) * DRLMI
          ESL(2) = DRL(2) * DRLMI
          ESL(3) = DRL(3) * DRLMI
C
          RROT(1) = 0.5*(RL(1,L2)+RL(1,L1)) - XYZREF(1)
          RROT(2) = 0.5*(RL(2,L2)+RL(2,L1)) - XYZREF(2)
          RROT(3) = 0.5*(RL(3,L2)+RL(3,L1)) - XYZREF(3)
C
C-------- go over freestream velocity and rotation components
          CALL CROSS(RROT,WROT,VROT)
C
          VEFF(1) = (VINF(1) + VROT(1))/BETM
          VEFF(2) =  VINF(2) + VROT(2)
          VEFF(3) =  VINF(3) + VROT(3)
C
C-------- set VEFF sensitivities to freestream,rotation components
          DO K = 1, 3
            VEFF_U(1,K) = 0.
            VEFF_U(2,K) = 0.
            VEFF_U(3,K) = 0.
            VEFF_U(K,K) = 1.0
          ENDDO
C
          DO K = 4, 6
            WROT_U(1) = 0.
            WROT_U(2) = 0.
            WROT_U(3) = 0.
            WROT_U(K-3) = 1.0
            CALL CROSS(RROT,WROT_U,VROT_U)
C
            VEFF_U(1,K) = VROT_U(1)
            VEFF_U(2,K) = VROT_U(2)
            VEFF_U(3,K) = VROT_U(3)
          ENDDO
C
C-------- U.es
          US = VEFF(1)*ESL(1) + VEFF(2)*ESL(2) + VEFF(3)*ESL(3)
C
C
C-------- velocity projected on normal plane = U - (U.es) es
          DO K = 1, 3
            UN = VEFF(K) - US*ESL(K)
            FB(K) = UN*SRC(L)
C
            DO IU = 1, 6
              UN_U = VEFF_U(K,IU)
     &             - ( VEFF_U(1,IU)*ESL(1)
     &                +VEFF_U(2,IU)*ESL(2)
     &                +VEFF_U(3,IU)*ESL(3))*ESL(K)
              FB_U(K,IU) = UN *SRC_U(L,IU) + UN_U*SRC(L)
            ENDDO
C
            DCPB(K,L) = FB(K) * 2.0 * DINV*DRLMI
          ENDDO
C
          CALL CROSS(RROT,FB,MB)
          DO IU = 1, 6
            CALL CROSS(RROT,FB_U(1,IU),MB_U(1,IU)) 
          ENDDO
C
          CDBDY(IB) = CDBDY(IB) + ( FB(1)*COSA + FB(3)*SINA) * 2.0/SREF
          CLBDY(IB) = CLBDY(IB) + (-FB(1)*SINA + FB(3)*COSA) * 2.0/SREF

          CXBDY(IB) = CXBDY(IB) +   FB(1) * 2.0/SREF
          CYBDY(IB) = CYBDY(IB) +   FB(2) * 2.0/SREF
          CZBDY(IB) = CZBDY(IB) +   FB(3) * 2.0/SREF
C
          CRBDY(IB) = CRBDY(IB) +   MB(1) * 2.0/SREF / BREF
          CMBDY(IB) = CMBDY(IB) +   MB(2) * 2.0/SREF / CREF
          CNBDY(IB) = CNBDY(IB) +   MB(3) * 2.0/SREF / BREF
C
          DO IU = 1, 6
            CDBDY_U(IU) = CDBDY_U(IU) + ( FB_U(1,IU)*COSA
     &                                  + FB_U(3,IU)*SINA) * 2.0/SREF
            CLBDY_U(IU) = CLBDY_U(IU) + (-FB_U(1,IU)*SINA
     &                                  + FB_U(3,IU)*COSA) * 2.0/SREF
c
            CXBDY_U(IU) = CXBDY_U(IU) +   FB_U(1,IU) * 2.0/SREF
            CYBDY_U(IU) = CYBDY_U(IU) +   FB_U(2,IU) * 2.0/SREF
            CZBDY_U(IU) = CZBDY_U(IU) +   FB_U(3,IU) * 2.0/SREF
C
            CRBDY_U(IU) = CRBDY_U(IU) +   MB_U(1,IU) * 2.0/SREF / BREF
            CMBDY_U(IU) = CMBDY_U(IU) +   MB_U(2,IU) * 2.0/SREF / CREF
            CNBDY_U(IU) = CNBDY_U(IU) +   MB_U(3,IU) * 2.0/SREF / BREF
          ENDDO
 205    CONTINUE
C
C---- add body forces and sensitivities to totals
        CDTOT = CDTOT + CDBDY(IB)
        CLTOT = CLTOT + CLBDY(IB)
C
        CXTOT = CXTOT + CXBDY(IB)
        CYTOT = CYTOT + CYBDY(IB)
        CZTOT = CZTOT + CZBDY(IB)
C
        CRTOT = CRTOT + CRBDY(IB)
        CMTOT = CMTOT + CMBDY(IB)
        CNTOT = CNTOT + CNBDY(IB)
C
        DO IU = 1, 6
          CDTOT_U(IU) = CDTOT_U(IU) + CDBDY_U(IU)
          CLTOT_U(IU) = CLTOT_U(IU) + CLBDY_U(IU)
C
          CXTOT_U(IU) = CXTOT_U(IU) + CXBDY_U(IU)
cccc      CXTOT_U(IU) = CYTOT_U(IU) + CYBDY_U(IU)   <<< BUG  5 Dec 10  MD
          CYTOT_U(IU) = CYTOT_U(IU) + CYBDY_U(IU)
          CZTOT_U(IU) = CZTOT_U(IU) + CZBDY_U(IU)
C
          CRTOT_U(IU) = CRTOT_U(IU) + CRBDY_U(IU)
          CMTOT_U(IU) = CMTOT_U(IU) + CMBDY_U(IU)
          CNTOT_U(IU) = CNTOT_U(IU) + CNBDY_U(IU)
        ENDDO
 200  CONTINUE
C
      RETURN
      END ! BDFORC



      SUBROUTINE VINFAB
C
C...Purpose:  To calculate free stream vector components and sensitivities
C
C...Input:   ALFA       Angle of attack (for stability-axis definition)
C            BETA       Sideslip angle (positive wind on right cheek facing fwd)
C...Output:  VINF(3)    Velocity components of free stream
C            VINF_A(3)  dVINF()/dALFA
C            VINF_B(3)  dVINF()/dBETA
C
      INCLUDE 'AVL.INC'
C
      SINA = SIN(ALFA)
      COSA = COS(ALFA)
      SINB = SIN(BETA)
      COSB = COS(BETA)
C
      VINF(1) =  COSA*COSB
      VINF(2) =      -SINB
      VINF(3) =  SINA*COSB
C
      VINF_A(1) = -SINA*COSB
      VINF_A(2) =  0.
      VINF_A(3) =  COSA*COSB
C
      VINF_B(1) = -COSA*SINB
      VINF_B(2) =      -COSB
      VINF_B(3) = -SINA*SINB
C
      RETURN
      END ! VINFAB
