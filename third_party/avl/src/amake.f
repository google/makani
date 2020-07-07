C***********************************************************************
C    Module:  amake.f
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

      SUBROUTINE MAKESURF(ISURF, IBX,NSEC, 
     &       NVC1, CSPACE, NVS1, SSPACE,
     &       XYZSCAL,XYZTRAN,ADDINC,
     &       XYZLES,CHORDS,AINCS,SSPACES,NSPANS,
     &       XASEC,SASEC,TASEC,NASEC,
     &       CLCDSEC,CLAF,
     &       ICONX, 
     &       ICONTD,NSCON,GAIND,XHINGED,VHINGED,REFLD,
     &       IDESTD,NSDES,GAING )
C--------------------------------------------------------------
C     Sets up all stuff for surface ISURF, 
C     using info from configuration input file.
C--------------------------------------------------------------
      INCLUDE 'AVL.INC'
C
      REAL XYZSCAL(3), XYZTRAN(3), ADDINC
      REAL XYZLES(3,*),CHORDS(*),AINCS(*),SSPACES(*)
      INTEGER NSPANS(*), NASEC(*)
      REAL XASEC(IBX,*), SASEC(IBX,*), TASEC(IBX,*)
      REAL CLCDSEC(6,*), CLAF(*)
      INTEGER ICONTD(ICONX,*),NSCON(*),
     &        IDESTD(ICONX,*),NSDES(*)
      REAL GAIND(ICONX,*),
     &     XHINGED(ICONX,*),VHINGED(3,ICONX,*),REFLD(ICONX,*),
     &     GAING(ICONX,*)
C
      REAL XYZLEL(3), XYZLER(3)
C
      PARAMETER (KCMAX=50,
     &           KSMAX=500)
      REAL XPT0(KCMAX), XCP0(KCMAX), XVR0(KCMAX), XSR0(KCMAX),
     &     XPT1(KCMAX), XCP1(KCMAX), XVR1(KCMAX), XSR1(KCMAX),
     &     XPT2(KCMAX), XCP2(KCMAX), XVR2(KCMAX), XSR2(KCMAX)
      REAL XPT(KCMAX), XCP(KCMAX), XVR(KCMAX), XSR(KCMAX),
     &     YPT(KSMAX), YCP(KSMAX)
      REAL YZLEN(KSMAX)
      INTEGER IPTLOC(KSMAX)
C
      PARAMETER (KPMAX=2*KCMAX+2*KSMAX)
      REAL FSPACE(KPMAX)
C
      REAL CHSINL_G(NGMAX),CHCOSL_G(NGMAX),
     &     CHSINR_G(NGMAX),CHCOSR_G(NGMAX)
      INTEGER ISCONL(NDMAX), ISCONR(NDMAX)
      REAL XLED(NDMAX), XTED(NDMAX), GAINDA(NDMAX)
C
C
      IF(NSEC.LT.2) THEN
       WRITE(*,*) '*** Need at least 2 sections per surface.'
       STOP
      ENDIF
C
      NVC = NVC1
      NVS = NVS1
C
      IF(NVC.GT.KCMAX) THEN
       WRITE(*,*) '* MAKESURF: Array overflow.  Increase KCMAX to', NVC
       NVC = KCMAX
      ENDIF
C
      IF(NVS.GT.KSMAX) THEN
       WRITE(*,*) '* MAKESURF: Array overflow.  Increase KSMAX to', NVS
       NVS = KSMAX
      ENDIF
C
C--- Image flag set to indicate section definition direction
C    IMAGS= 1  defines edge 1 located at surface root edge 
C    IMAGS=-1  defines edge 2 located at surface root edge (reflected surfaces)
      IMAGS(ISURF) = 1
      IFRST(ISURF) = NVOR   + 1 
      JFRST(ISURF) = NSTRIP + 1
      NK(ISURF) = NVC
C
C-----------------------------------------------------------------
C---- section arc lengths of wing trace in y-z plane
      YZLEN(1) = 0.
      DO ISEC = 2, NSEC
        DY = XYZLES(2,ISEC) - XYZLES(2,ISEC-1)
        DZ = XYZLES(3,ISEC) - XYZLES(3,ISEC-1)
        YZLEN(ISEC) = YZLEN(ISEC-1) + SQRT(DY*DY + DZ*DZ)
      ENDDO
C
C
      IF(NVS1.EQ.0) THEN
C----- set spanwise spacing using spacing parameters for each section interval
       NVS = 0
       DO ISEC = 1, NSEC-1
         NVS = NVS + NSPANS(ISEC)
       ENDDO
       IF(NVS.GT.KSMAX) THEN
        WRITE(*,*) '*** MAKESURF: Array overflow. Increase KSMAX to',NVS
        STOP
       ENDIF
C
       NVS = 0
       YPT(1) = YZLEN(1)
       IPTLOC(1) = 1
C
       DO ISEC = 1, NSEC-1
         DYZLEN = YZLEN(ISEC+1) - YZLEN(ISEC)
C
         NVINT = NSPANS(ISEC)
C
C------- set spanwise spacing array
         NSPACE = 2*NVINT + 1
         IF(NSPACE.GT.KPMAX) THEN
          WRITE(*,*) '*** MAKESURF: Array overflow. Increase KPMAX to', 
     &                 NSPACE
          STOP
         ENDIF
         CALL SPACER(NSPACE,SSPACES(ISEC),FSPACE)
C
         DO N = 1, NVINT
           IVS = NVS + N
           YCP(IVS)   = YPT(NVS+1) + DYZLEN*FSPACE(2*N)
           YPT(IVS+1) = YPT(NVS+1) + DYZLEN*FSPACE(2*N+1)
         ENDDO
         IPTLOC(ISEC+1) = NVS + NVINT + 1
C
         NVS = NVS + NVINT
       ENDDO
C
      ELSE
C----- set spanwise spacing using overall parameters NVS, SSPACE
C
       NSPACE = 2*NVS + 1
       IF(NSPACE.GT.KPMAX) THEN
        WRITE(*,*) '*** MAKESURF: Array overflow. Increase KPMAX to', 
     &              NSPACE
        STOP
       ENDIF
       CALL SPACER(NSPACE,SSPACE,FSPACE)
C
       YPT(1) = YZLEN(1)
       DO IVS = 1, NVS
         YCP(IVS)   = YZLEN(1) + (YZLEN(NSEC)-YZLEN(1))*FSPACE(2*IVS)
         YPT(IVS+1) = YZLEN(1) + (YZLEN(NSEC)-YZLEN(1))*FSPACE(2*IVS+1)
       ENDDO
C
       NPT = NVS + 1
C
C----- find node nearest each section
       DO ISEC = 2, NSEC-1
         YPTLOC = 1.0E9
         IPTLOC(ISEC) = 1
         DO IPT = 1, NPT
           YPTDEL = ABS(YZLEN(ISEC) - YPT(IPT))
           IF(YPTDEL .LT. YPTLOC) THEN
            YPTLOC = YPTDEL
            IPTLOC(ISEC) = IPT
           ENDIF
         ENDDO
       ENDDO
       IPTLOC(1)    = 1
       IPTLOC(NSEC) = NPT
C
C----- fudge Glauert angles to make nodes match up exactly with interior sections
       DO ISEC = 2, NSEC-1
         IPT1 = IPTLOC(ISEC-1)
         IPT2 = IPTLOC(ISEC  )
         IF(IPT1.EQ.IPT2) THEN
          CALL STRIP(STITLE(ISURF),NST)
          WRITE(*,7000) ISEC, STITLE(ISURF)(1:NST)
          STOP
         ENDIF
C
         YPT1 = YPT(IPT1)
         YSCALE = (YZLEN(ISEC)-YZLEN(ISEC-1)) / (YPT(IPT2)-YPT(IPT1))
         DO IPT = IPT1, IPT2-1
           YPT(IPT) = YZLEN(ISEC-1) + YSCALE*(YPT(IPT)-YPT1)
         ENDDO
         DO IVS = IPT1, IPT2-1
           YCP(IVS) = YZLEN(ISEC-1) + YSCALE*(YCP(IVS)-YPT1)
         ENDDO
C
         IPT1 = IPTLOC(ISEC  )
         IPT2 = IPTLOC(ISEC+1)
         IF(IPT1.EQ.IPT2) THEN
          CALL STRIP(STITLE(ISURF),NST)
          WRITE(*,7000) ISEC, STITLE(ISURF)(1:NST)
          STOP
         ENDIF
C
         YPT1 = YPT(IPT1)
         YSCALE = (YPT(IPT2)-YZLEN(ISEC)) / (YPT(IPT2)-YPT(IPT1))
         DO IPT = IPT1, IPT2-1
           YPT(IPT) = YZLEN(ISEC) + YSCALE*(YPT(IPT)-YPT1)
         ENDDO
         DO IVS = IPT1, IPT2-1
           YCP(IVS) = YZLEN(ISEC) + YSCALE*(YCP(IVS)-YPT1)
         ENDDO
C
 7000    FORMAT(
     &   /' *** Cannot adjust spanwise spacing at section', I3, 
     &    ', on surface ', A
     &   /' *** Insufficient number of spanwise vortices to work with')
       ENDDO
C
      ENDIF
C
C
C====================================================
C---- define strips between input sections
C
      NJ(ISURF) = 0
C
      IF(NCONTROL.GT.NDMAX) THEN
       WRITE(*,*) '*** Too many control variables.  Increase NDMAX to',
     &            NCONTROL
       STOP
      ENDIF
C
      IF(NDESIGN.GT.NGMAX) THEN
       WRITE(*,*) '*** Too many design variables.  Increase NGMAX to',
     &            NDESIGN
       STOP
      ENDIF
C
C---- go over section intervals
      DO 200 ISEC = 1, NSEC-1
        XYZLEL(1) = XYZSCAL(1)*XYZLES(1,ISEC)    + XYZTRAN(1)
        XYZLEL(2) = XYZSCAL(2)*XYZLES(2,ISEC)    + XYZTRAN(2)
        XYZLEL(3) = XYZSCAL(3)*XYZLES(3,ISEC)    + XYZTRAN(3)
        XYZLER(1) = XYZSCAL(1)*XYZLES(1,ISEC+1)  + XYZTRAN(1)
        XYZLER(2) = XYZSCAL(2)*XYZLES(2,ISEC+1)  + XYZTRAN(2)
        XYZLER(3) = XYZSCAL(3)*XYZLES(3,ISEC+1)  + XYZTRAN(3)
C
        WIDTH = SQRT(  (XYZLER(2)-XYZLEL(2))**2
     &               + (XYZLER(3)-XYZLEL(3))**2 )
C
        CHORDL = XYZSCAL(1)*CHORDS(ISEC)
        CHORDR = XYZSCAL(1)*CHORDS(ISEC+1)
C
        CLAFL = CLAF(ISEC)
        CLAFR = CLAF(ISEC+1)
C
C------ removed CLAF influence on zero-lift angle  (MD  21 Mar 08)
        AINCL = AINCS(ISEC)   + ADDINC
        AINCR = AINCS(ISEC+1) + ADDINC
cc      AINCL = AINCS(ISEC)   + ADDINC - 4.0*DTR*(CLAFL-1.0)
cc      AINCR = AINCS(ISEC+1) + ADDINC - 4.0*DTR*(CLAFR-1.0)
C
        CHSINL = CHORDL*SIN(AINCL)
        CHSINR = CHORDR*SIN(AINCR)
        CHCOSL = CHORDL*COS(AINCL)
        CHCOSR = CHORDR*COS(AINCR)
C
C------ set control-declaration lines for each control variable
        DO N = 1, NCONTROL
          ISCONL(N) = 0
          ISCONR(N) = 0
          DO ISCON = 1, NSCON(ISEC)
            IF(ICONTD(ISCON,ISEC)  .EQ.N) ISCONL(N) = ISCON
          ENDDO
          DO ISCON = 1, NSCON(ISEC+1)
            IF(ICONTD(ISCON,ISEC+1).EQ.N) ISCONR(N) = ISCON
          ENDDO
        ENDDO
C
C------ set design-variable sensitivities of CHSIN and CHCOS
        DO N = 1, NDESIGN
          CHSINL_G(N) = 0.
          CHSINR_G(N) = 0.
          CHCOSL_G(N) = 0.
          CHCOSR_G(N) = 0.
C
          DO ISDES = 1, NSDES(ISEC)
            IF(IDESTD(ISDES,ISEC).EQ.N) THEN
             CHSINL_G(N) =  CHCOSL * GAING(ISDES,ISEC)*DTR
             CHCOSL_G(N) = -CHSINL * GAING(ISDES,ISEC)*DTR
            ENDIF
          ENDDO
C
          DO ISDES = 1, NSDES(ISEC+1)
            IF(IDESTD(ISDES,ISEC+1).EQ.N) THEN
             CHSINR_G(N) =  CHCOSR * GAING(ISDES,ISEC+1)*DTR
             CHCOSR_G(N) = -CHSINR * GAING(ISDES,ISEC+1)*DTR
            ENDIF
          ENDDO
        ENDDO
C
C
C------ go over chord strips
        IPTL = IPTLOC(ISEC)
        IPTR = IPTLOC(ISEC+1)
        NSPAN = IPTR - IPTL
        DO 150 ISPAN = 1, NSPAN
C-------- define left and right edges of vortex strip
C-          note that incidence angle is set by ATAN of chord projections,
C-          not by linear interpolation of AINC
          IPT1 = IPTL + ISPAN - 1
          IPT2 = IPTL + ISPAN
          IVS  = IPTL + ISPAN - 1
          F1 = (YPT(IPT1)-YPT(IPTL))/(YPT(IPTR)-YPT(IPTL))
          F2 = (YPT(IPT2)-YPT(IPTL))/(YPT(IPTR)-YPT(IPTL))
          FC = (YCP(IVS) -YPT(IPTL))/(YPT(IPTR)-YPT(IPTL))
C
C-------- store strip in global data arrays
          NSTRIP = NSTRIP + 1
          NJ(ISURF) = NJ(ISURF) + 1
C
          RLE1(1,NSTRIP) = (1.0-F1)*XYZLEL(1) + F1*XYZLER(1)
          RLE1(2,NSTRIP) = (1.0-F1)*XYZLEL(2) + F1*XYZLER(2)
          RLE1(3,NSTRIP) = (1.0-F1)*XYZLEL(3) + F1*XYZLER(3)
          CHORD1(NSTRIP) = (1.0-F1)*CHORDL    + F1*CHORDR
C
          RLE2(1,NSTRIP) = (1.0-F2)*XYZLEL(1) + F2*XYZLER(1)
          RLE2(2,NSTRIP) = (1.0-F2)*XYZLEL(2) + F2*XYZLER(2)
          RLE2(3,NSTRIP) = (1.0-F2)*XYZLEL(3) + F2*XYZLER(3)
          CHORD2(NSTRIP) = (1.0-F2)*CHORDL    + F2*CHORDR
C
          RLE(1,NSTRIP)  = (1.0-FC)*XYZLEL(1) + FC*XYZLER(1)
          RLE(2,NSTRIP)  = (1.0-FC)*XYZLEL(2) + FC*XYZLER(2)
          RLE(3,NSTRIP)  = (1.0-FC)*XYZLEL(3) + FC*XYZLER(3)
          CHORD(NSTRIP)  = (1.0-FC)*CHORDL    + FC*CHORDR
C
          WSTRIP(NSTRIP) = ABS(F2-F1)*WIDTH
          TANLE(NSTRIP)  = (XYZLER(1)-XYZLEL(1))/WIDTH
          TANTE(NSTRIP)  = (XYZLER(1)+CHORDR - XYZLEL(1)-CHORDL)/WIDTH
C
          CHSIN = CHSINL + FC*(CHSINR-CHSINL)
          CHCOS = CHCOSL + FC*(CHCOSR-CHCOSL)
          AINC(NSTRIP) = ATAN2(CHSIN,CHCOS)
C
          DO N = 1, NDESIGN
            CHSIN_G = (1.0-FC)*CHSINL_G(N) + FC*CHSINR_G(N)
            CHCOS_G = (1.0-FC)*CHCOSL_G(N) + FC*CHCOSR_G(N)
            AINC_G(NSTRIP,N) = (CHCOS*CHSIN_G - CHSIN*CHCOS_G)
     &                       / (CHSIN**2 + CHCOS**2)
          ENDDO
C
          DO N = 1, NCONTROL
            ICL = ISCONL(N)
            ICR = ISCONR(N)
C
            IF(ICL.EQ.0 .OR. ICR.EQ.0) THEN
C----------- no control effect here
             GAINDA(N) = 0.
             XLED(N) = 0.
             XTED(N) = 0.
C
             VHINGE(1,NSTRIP,N) = 0.
             VHINGE(2,NSTRIP,N) = 0.
             VHINGE(3,NSTRIP,N) = 0.
C
             VREFL(NSTRIP,N) = 0.
C
             PHINGE(1,NSTRIP,N) = 0.
             PHINGE(2,NSTRIP,N) = 0.
             PHINGE(3,NSTRIP,N) = 0.
C
            ELSE
C----------- control variable # N is active here
             GAINDA(N) = GAIND(ICL,ISEC  )*(1.0-FC)
     &                 + GAIND(ICR,ISEC+1)*     FC
C
             XHD = CHORDL*XHINGED(ICL,ISEC  )*(1.0-FC)
     &           + CHORDR*XHINGED(ICR,ISEC+1)*     FC
             IF(XHD.GE.0.0) THEN
C------------ TE control surface, with hinge at XHD
              XLED(N) = XHD
              XTED(N) = CHORD(NSTRIP)
             ELSE
C------------ LE control surface, with hinge at -XHD
              XLED(N) =  0.0
              XTED(N) = -XHD
             ENDIF
C
             VHX = VHINGED(1,ICL,ISEC)*XYZSCAL(1)
             VHY = VHINGED(2,ICL,ISEC)*XYZSCAL(2)
             VHZ = VHINGED(3,ICL,ISEC)*XYZSCAL(3)
             VSQ = VHX**2 + VHY**2 + VHZ**2
             IF(VSQ.EQ.0.0) THEN
C------------ default: set hinge vector along hingeline
              VHX = XYZLES(1,ISEC+1) + ABS(CHORDR*XHINGED(ICR,ISEC+1))
     &            - XYZLES(1,ISEC  ) - ABS(CHORDL*XHINGED(ICL,ISEC  ))
              VHY = XYZLES(2,ISEC+1)
     &            - XYZLES(2,ISEC  )
              VHZ = XYZLES(3,ISEC+1)
     &            - XYZLES(3,ISEC  )
              VHX = VHX*XYZSCAL(1)
              VHY = VHY*XYZSCAL(2)
              VHZ = VHZ*XYZSCAL(3)
              VSQ = VHX**2 + VHY**2 + VHZ**2
             ENDIF
C
             VMOD = SQRT(VSQ)
             VHINGE(1,NSTRIP,N) = VHX/VMOD
             VHINGE(2,NSTRIP,N) = VHY/VMOD
             VHINGE(3,NSTRIP,N) = VHZ/VMOD
C
             VREFL(NSTRIP,N) = REFLD(ICL,ISEC)
C
             IF(XHD .GE. 0.0) THEN
              PHINGE(1,NSTRIP,N) = RLE(1,NSTRIP) + XHD
              PHINGE(2,NSTRIP,N) = RLE(2,NSTRIP)
              PHINGE(3,NSTRIP,N) = RLE(3,NSTRIP)
             ELSE
              PHINGE(1,NSTRIP,N) = RLE(1,NSTRIP) - XHD
              PHINGE(2,NSTRIP,N) = RLE(2,NSTRIP)
              PHINGE(3,NSTRIP,N) = RLE(3,NSTRIP)
             ENDIF
C
            ENDIF
          ENDDO
C
C--- Interpolate CD-CL polar defining data from input sections to strips
          DO L = 1, 6
            CLCD(L,NSTRIP) = (1.0-FC)*CLCDSEC(L,ISEC) 
     &                      +     FC *CLCDSEC(L,ISEC+1)
          END DO
C--- If the min drag is zero flag the strip as no-viscous data
          LVISCSTRP(NSTRIP) = (CLCD(4,NSTRIP).NE.0.0)
C
C
          IJFRST(NSTRIP) = NVOR + 1
          NVSTRP(NSTRIP) = NVC
C
          NSURFS(NSTRIP) = ISURF
C
          NSL = NASEC(ISEC  )
          NSR = NASEC(ISEC+1)
C
          CHORDC = CHORD(NSTRIP)
C
          CLAFC =  (1.-FC)*(CHORDL/CHORDC)*CLAFL
     &           +     FC *(CHORDR/CHORDC)*CLAFR
C
C-------- set chordwise spacing fraction arrays
          CALL CSPACER(NVC,CSPACE,CLAFC, XPT,XVR,XSR,XCP)
c
C-------- go over vortices in this strip
          DO 1505 IVC = 1, NVC
            NVOR = NVOR + 1
C
            RV1(1,NVOR) = RLE1(1,NSTRIP) + XVR(IVC)*CHORD1(NSTRIP)
            RV1(2,NVOR) = RLE1(2,NSTRIP)
            RV1(3,NVOR) = RLE1(3,NSTRIP)
C
            RV2(1,NVOR) = RLE2(1,NSTRIP) + XVR(IVC)*CHORD2(NSTRIP)
            RV2(2,NVOR) = RLE2(2,NSTRIP)
            RV2(3,NVOR) = RLE2(3,NSTRIP)
C
            RV(1,NVOR) = RLE(1,NSTRIP) + XVR(IVC)*CHORDC
            RV(2,NVOR) = RLE(2,NSTRIP)
            RV(3,NVOR) = RLE(3,NSTRIP)
C
            RC(1,NVOR) = RLE(1,NSTRIP) + XCP(IVC)*CHORDC
            RC(2,NVOR) = RLE(2,NSTRIP)
            RC(3,NVOR) = RLE(3,NSTRIP)
C
            RS(1,NVOR) = RLE(1,NSTRIP) + XSR(IVC)*CHORDC
            RS(2,NVOR) = RLE(2,NSTRIP)
            RS(3,NVOR) = RLE(3,NSTRIP)
C
            CALL AKIMA(XASEC(1,ISEC  ),SASEC(1,ISEC  ),NSL,
     &                 XCP(IVC),SLOPEL, DSDX)
            CALL AKIMA(XASEC(1,ISEC+1),SASEC(1,ISEC+1),NSR,
     &                 XCP(IVC),SLOPER, DSDX)
            SLOPEC(NVOR) =  (1.-FC)*(CHORDL/CHORDC)*SLOPEL 
     &                    +     FC *(CHORDR/CHORDC)*SLOPER
C
            CALL AKIMA(XASEC(1,ISEC  ),SASEC(1,ISEC  ),NSL,
     &                 XVR(IVC),SLOPEL, DSDX)
            CALL AKIMA(XASEC(1,ISEC+1),SASEC(1,ISEC+1),NSR,
     &                 XVR(IVC),SLOPER, DSDX)
            SLOPEV(NVOR) =  (1.-FC)*(CHORDL/CHORDC)*SLOPEL 
     &                    +     FC *(CHORDR/CHORDC)*SLOPER
C
            DXOC = XPT(IVC+1) - XPT(IVC)
            DXV(NVOR) = DXOC*CHORDC
            CHORDV(NVOR) = CHORDC
            NSURFV(NVOR) = LSCOMP(ISURF)

            LVNC(NVOR) = .TRUE.
C
C---------- element inherits alpha,beta flag from surface
            LVALBE(NVOR) = LFALBE(ISURF)
C
            DO N = 1, NCONTROL
C------------ scale control gain by factor 0..1, (fraction of element on control surface)
              FRACLE = (XLED(N)/CHORDC-XPT(IVC)) / DXOC
              FRACTE = (XTED(N)/CHORDC-XPT(IVC)) / DXOC
C
              FRACLE = MIN( 1.0 , MAX( 0.0 , FRACLE ) )
              FRACTE = MIN( 1.0 , MAX( 0.0 , FRACTE ) )
C
              DCONTROL(NVOR,N) = GAINDA(N)*(FRACTE-FRACLE)
            ENDDO
C
C---------- TE control point used only if surface sheds a wake
            LVNC(NVOR) = LFWAKE(ISURF)

 1505     CONTINUE
C
 150    CONTINUE
C
 200  CONTINUE
C
C---- Find wetted surface area (one side)
      SUM  = 0.0
      WTOT = 0.0
      DO JJ = 1, NJ(ISURF)
        J = JFRST(ISURF) + JJ-1 
        ASTRP = WSTRIP(J)*CHORD(J)
        SUM  = SUM + ASTRP
        WTOT = WTOT + WSTRIP(J)
      ENDDO
      SSURF(ISURF) = SUM
C
      IF(WTOT .EQ. 0.0) THEN
       CAVESURF(ISURF) = 0.0
      ELSE
       CAVESURF(ISURF) = SUM/WTOT
      ENDIF
C
      RETURN
      END ! MAKESURF



      SUBROUTINE MAKEBODY(IBODY, IBX,
     &       NVB1, BSPACE,
     &       XYZSCAL,XYZTRAN,
     &       XBOD,YBOD,TBOD,NBOD)
C--------------------------------------------------------------
C     Sets up all stuff for body IBODY,
C     using info from configuration input file.
C--------------------------------------------------------------
      INCLUDE 'AVL.INC'
C
      REAL XYZSCAL(3), XYZTRAN(3)
      REAL XBOD(IBX), YBOD(IBX), TBOD(IBX)
C
      PARAMETER (KLMAX=101)
      REAL XPT(KLMAX), FSPACE(KLMAX)
C
C
c      IF(NSEC.LT.2) THEN
c       WRITE(*,*) '*** Need at least 2 sections per body.'
c       STOP
c      ENDIF
C
      NVB = NVB1
C
      IF(NVB.GT.KLMAX) THEN
       WRITE(*,*) '* MAKEBODY: Array overflow.  Increase KLMAX to', NVB
       NVB = KLMAX
      ENDIF
C
C
      LFRST(IBODY) = NLNODE + 1 
      NL(IBODY) = NVB
C
      IF(NLNODE+NVB.GT.NLMAX) THEN
       WRITE(*,*) '*** MAKEBODY: Array overflow. Increase NLMAX to',
     &             NLNODE+NVB
       STOP
      ENDIF
C
C-----------------------------------------------------------------
C---- set lengthwise spacing fraction arrays
      NSPACE = NVB + 1
      IF(NSPACE.GT.KLMAX) THEN
       WRITE(*,*) '*** MAKEBODY: Array overflow. Increase KLMAX to', 
     &             NSPACE
       STOP
      ENDIF
      CALL SPACER(NSPACE,BSPACE,FSPACE)
C
      DO IVB = 1, NVB
        XPT(IVB) = FSPACE(IVB)
      ENDDO
      XPT(1) = 0.0
      XPT(NVB+1) = 1.0
C
C---- set body nodes and radii
      VOLB = 0.0
      SRFB = 0.0
      DO IVB = 1, NVB+1
        NLNODE = NLNODE + 1
C
        XVB = XBOD(1) + (XBOD(NBOD)-XBOD(1))*XPT(IVB)
        CALL AKIMA(XBOD,YBOD,NBOD,XVB,YVB,DYDX)
        RL(1,NLNODE) = XYZTRAN(1) + XYZSCAL(1)*XVB
        RL(2,NLNODE) = XYZTRAN(2)
        RL(3,NLNODE) = XYZTRAN(3) + XYZSCAL(3)*YVB
C
        CALL AKIMA(XBOD,TBOD,NBOD,XVB,TVB,DRDX)
        RADL(NLNODE) = SQRT(XYZSCAL(2)*XYZSCAL(3)) * 0.5*TVB
      ENDDO
C---- get surface length, area and volume
      VOLB = 0.0
      SRFB = 0.0
      XBMN = RL(1,LFRST(IBODY))
      XBMX = XBMN
      DO IVB = 1, NVB
        NL0 = LFRST(IBODY) + IVB-1
        NL1 = NL0 + 1
        X0 = RL(1,NL0)
        X1 = RL(1,NL1)
        DX = ABS(X1 - X0)
        R0 = RADL(NL0)
        R1 = RADL(NL1)
        DVOL = PI*DX * (R0**2 + R0*R1 + R1**2) / 3.0
        DS = SQRT((R0-R1)**2 + DX**2)
        DSRF = PI*DS * (R0+R1)
C
        SRFB = SRFB + DSRF
        VOLB = VOLB + DVOL
        XBMN = MIN(XBMN,X0,X1)
        XBMX = MAX(XBMX,X0,X1)
      ENDDO
      VOLBDY(IBODY) = VOLB
      SRFBDY(IBODY) = SRFB
      ELBDY(IBODY) = XBMX - XBMN
C
      RETURN
      END ! MAKEBODY




      SUBROUTINE SDUPL(NN,YDUPL,MSG)
C-----------------------------------
C     Adds image of surface NN,
C     reflected about y=YDUPL.
C-----------------------------------
      INCLUDE 'AVL.INC'
      CHARACTER*(*) MSG
C
      NNI = NSURF + 1
      IF(NNI.GT.NFMAX) THEN
        WRITE(*,*) 'SDUPL: Surface array overflow. Increase NFMAX.'
        STOP
      ENDIF
C
      KLEN = LEN(STITLE(NN))
      DO K = KLEN, 1, -1
        IF(STITLE(NN)(K:K) .NE. ' ') GO TO 6
      ENDDO
 6    STITLE(NNI) = STITLE(NN)(1:K) // ' (' // MSG // ')'
      WRITE(*,*) ' '
      WRITE(*,*) '  Building duplicate image-surface: ',STITLE(NNI)
C
C---- duplicate surface is assumed to be the same logical component surface
      LSCOMP(NNI) = LSCOMP(NN)
C
C---- same various logical flags
      LFWAKE(NNI) = LFWAKE(NN)
      LFALBE(NNI) = LFALBE(NN)
      LFLOAD(NNI) = LFLOAD(NN)

C---- accumulate stuff for new image surface 
      IFRST(NNI) = NVOR   + 1
      JFRST(NNI) = NSTRIP + 1
      NJ(NNI) = NJ(NN)
      NK(NNI) = NK(NN)
C
      NVC = NK(NNI)
      NVS = NJ(NNI)
C
      SSURF(NNI)    = SSURF(NN)
      CAVESURF(NNI) = CAVESURF(NN)
C--- Note hinge axis is flipped to reverse the Y component of the hinge
C    vector.   This means that deflections need to be reversed for image
C    surfaces.
C
C--- Image flag reversed (set to -IMAGS) for imaged surfaces
      IMAGS(NNI) = -IMAGS(NN)
C
      YOFF = 2.0*YDUPL
C
C--- Create image strips, to maintain the same sense of positive GAMMA
C    these have the 1 and 2 strip edges reversed (i.e. root is edge 2, 
C    not edge 1 as for a strip with IMAGS=1
      DO 100 IVS = 1, NVS
        NSTRIP = NSTRIP + 1
        IF(NSTRIP.GT.NSMAX) THEN
          WRITE(*,*) 'SDUPL: Strip array overflow. Increase NSMAX.'
          STOP
        ENDIF
C
        JJI = JFRST(NNI) + IVS-1
        JJ  = JFRST(NN)  + IVS-1
        RLE1(1,JJI)   =  RLE2(1,JJ)
        RLE1(2,JJI)   = -RLE2(2,JJ) + YOFF
        RLE1(3,JJI)   =  RLE2(3,JJ)
        CHORD1(JJI) =  CHORD2(JJ)
        RLE2(1,JJI)   =  RLE1(1,JJ)
        RLE2(2,JJI)   = -RLE1(2,JJ) + YOFF
        RLE2(3,JJI)   =  RLE1(3,JJ)
        CHORD2(JJI) =  CHORD1(JJ)
        RLE(1,JJI)    =  RLE(1,JJ)
        RLE(2,JJI)    = -RLE(2,JJ) + YOFF
        RLE(3,JJI)    =  RLE(3,JJ)
        CHORD(JJI)  =  CHORD(JJ)
        WSTRIP(JJI) =  WSTRIP(JJ)
        TANLE(JJI)  = -TANLE(JJ)
        AINC (JJI)  =  AINC(JJ)
C
        NSURFS(NSTRIP) = NNI
C
        DO N = 1, NDESIGN
          AINC_G(JJI,N) = AINC_G(JJ,N)
        ENDDO
C
        DO N = 1, NCONTROL
          VREFL(JJI,N) = VREFL(JJ,N)
C
          VHINGE(1,JJI,N) =  VHINGE(1,JJ,N)
          VHINGE(2,JJI,N) = -VHINGE(2,JJ,N)
          VHINGE(3,JJI,N) =  VHINGE(3,JJ,N)
C
          PHINGE(1,JJI,N) =  PHINGE(1,JJ,N)
          PHINGE(2,JJI,N) = -PHINGE(2,JJ,N) + YOFF
          PHINGE(3,JJI,N) =  PHINGE(3,JJ,N)
        ENDDO
C
C--- The defined section for image strip is flagged with (-)
        IJFRST(JJI)  = NVOR + 1
        NVSTRP(JJI)  = NVC
        DO L = 1, 6
          CLCD(L,JJI) = CLCD(L,JJ) 
        END DO
        LVISCSTRP(JJI) = LVISCSTRP(JJ)
C
        DO 80 IVC = 1, NVC
          NVOR = NVOR + 1
          IF(NVOR.GT.NVMAX) THEN
            WRITE(*,*) 'SDUPL: Vortex array overflow. Increase NVMAX.'
            STOP
          ENDIF
C
          III = IJFRST(JJI) + IVC-1
          II  = IJFRST(JJ)  + IVC-1
          RV1(1,III)     =  RV2(1,II)
          RV1(2,III)     = -RV2(2,II) + YOFF
          RV1(3,III)     =  RV2(3,II)
          RV2(1,III)     =  RV1(1,II)
          RV2(2,III)     = -RV1(2,II) + YOFF
          RV2(3,III)     =  RV1(3,II)
          RV(1,III)     =  RV(1,II)
          RV(2,III)     = -RV(2,II) + YOFF
          RV(3,III)     =  RV(3,II)
          RC(1,III)     =  RC(1,II)
          RC(2,III)     = -RC(2,II) + YOFF
          RC(3,III)     =  RC(3,II)
          SLOPEC(III) = SLOPEC(II)
          SLOPEV(III) = SLOPEV(II)
          DXV(III)     = DXV(II)
          CHORDV(III) = CHORDV(II)
          NSURFV(III) = LSCOMP(NNI)
          LVALBE(III) = LVALBE(II)
          LVNC(III) = LVNC(II)
C
          DO N = 1, NCONTROL
ccc         RSGN = SIGN( 1.0 , VREFL(JJ,N) )
            RSGN = VREFL(JJ,N)
            DCONTROL(III,N) = -DCONTROL(II,N)*RSGN
          ENDDO
C
   80   CONTINUE
C
  100 CONTINUE
C
      NSURF = NSURF + 1
C
      RETURN
      END ! SDUPL




      SUBROUTINE BDUPL(NN,YDUPL,MSG)
C-----------------------------------
C     Adds image of surface NN,
C     reflected about y=YDUPL.
C-----------------------------------
      INCLUDE 'AVL.INC'
      CHARACTER*(*) MSG
C
      NNI = NBODY + 1
      IF(NNI.GT.NBMAX) THEN
        WRITE(*,*) 'BDUPL: Body array overflow. Increase NBMAX.'
        STOP
      ENDIF
C
      KLEN = LEN(BTITLE(NN))
      DO K = KLEN, 1, -1
        IF(BTITLE(NN)(K:K) .NE. ' ') GO TO 6
      ENDDO
 6    BTITLE(NNI) = BTITLE(NN)(1:K) // ' (' // MSG // ')'
      WRITE(*,*) ' '
      WRITE(*,*) '  Building duplicate image-body: ',BTITLE(NNI)
C
      LFRST(NNI) = NLNODE + 1
      NL(NNI) = NL(NN)
C
      NVB = NL(NNI)
C
      IF(NLNODE+NVB.GT.NLMAX) THEN
       WRITE(*,*) '*** MAKEBODY: Array overflow. Increase NLMAX to',
     &             NLNODE+NVB
       STOP
      ENDIF
C
C
      ELBDY(NNI)  = ELBDY(NN)
      SRFBDY(NNI) = SRFBDY(NN)
      VOLBDY(NNI) = VOLBDY(NN)
C
      YOFF = 2.0*YDUPL
C
C---- set body nodes and radii
      DO IVB = 1, NVB+1
        NLNODE = NLNODE + 1
C
        LLI = LFRST(NNI) + IVB-1
        LL  = LFRST(NN)  + IVB-1
C
        RL(1,LLI) =  RL(1,LL)
        RL(2,LLI) = -RL(2,LL) + YOFF
        RL(3,LLI) =  RL(3,LL)
C
        RADL(LLI) =  RADL(LL)
      ENDDO
C
      NBODY = NBODY + 1
C
      RETURN
      END ! BDUPL




      SUBROUTINE ENCALC
C
C...PURPOSE  To calculate the normal vectors for the strips, 
C            the horseshoe vortices, and the control points.
C            Incorporates surface deflections.
C
C...INPUT    NVOR      Number of vortices
C            X1        Coordinates of endpoint #1 of the vortices
C            X2        Coordinates of endpoint #2 of the vortices
C            SLOPEV    Slope at bound vortices
C            SLOPEC    Slope at control points
C            NSTRIP    Number of strips
C            IJFRST    Index of first element in strip
C            NVSTRP    No. of vortices in strip
C            AINC      Angle of incidence of strip
C            LDES      include design-variable deflections if TRUE
C
C...OUTPUT   ENC(3)        Normal vector at control point
C            ENV(3)        Normal vector at bound vortices
C            ENSY, ENSZ    Strip normal vector (ENSX=0)
C            LSTRIPOFF     Non-used strip (T) (below z=ZSYM)
C
C...COMMENTS   
C
      INCLUDE 'AVL.INC'
C
      REAL EP(3), EQ(3), ES(3), EB(3), EC(3), ECXB(3)
      REAL EC_G(3,NDMAX), ECXB_G(3)
C
C...Calculate the normal vector at control points and bound vortex midpoints
C
      DO 10 J = 1, NSTRIP
C
C...Calculate normal vector for the strip (normal to X axis)
        I = IJFRST(J)
        DXLE =  RV2(1,I)-RV1(1,I)
        DYLE =  RV2(2,I)-RV1(2,I)
        DZLE =  RV2(3,I)-RV1(3,I)
c       AXLE = (RV2(1,I)+RV1(1,I))*0.5
c       AYLE = (RV2(2,I)+RV1(2,I))*0.5
c       AZLE = (RV2(3,I)+RV1(3,I))*0.5
        AXLE = RV(1,I)
        AYLE = RV(2,I)
        AZLE = RV(3,I)
C
        I = IJFRST(J) + (NVSTRP(J)-1)
        DXTE =  RV2(1,I)-RV1(1,I)
        DYTE =  RV2(2,I)-RV1(2,I)
        DZTE =  RV2(3,I)-RV1(3,I)
c       AXTE = (RV2(1,I)+RV1(1,I))*0.5
c       AYTE = (RV2(2,I)+RV1(2,I))*0.5
c       AZTE = (RV2(3,I)+RV1(3,I))*0.5
        AXTE = RV(1,I)
        AYTE = RV(2,I)
        AZTE = RV(3,I)
C
        DXT = (1.0-SAXFR)*DXLE + SAXFR*DXTE
        DYT = (1.0-SAXFR)*DYLE + SAXFR*DYTE
        DZT = (1.0-SAXFR)*DZLE + SAXFR*DZTE
C
        ESS(1,J) =  DXT/SQRT(DXT*DXT + DYT*DYT + DZT*DZT)
        ESS(2,J) =  DYT/SQRT(DXT*DXT + DYT*DYT + DZT*DZT)
        ESS(3,J) =  DZT/SQRT(DXT*DXT + DYT*DYT + DZT*DZT)
C
        ENSY(J) = -DZT/SQRT(DYT*DYT + DZT*DZT)
        ENSZ(J) =  DYT/SQRT(DYT*DYT + DZT*DZT)
C
        XSREF(J) = (1.0-SAXFR)*AXLE + SAXFR*AXTE
        YSREF(J) = (1.0-SAXFR)*AYLE + SAXFR*AYTE
        ZSREF(J) = (1.0-SAXFR)*AZLE + SAXFR*AZTE
C
C
        ES(1) = 0.
        ES(2) = ENSY(J)
        ES(3) = ENSZ(J)
C
        LSTRIPOFF(J) = .FALSE.
C
        NV = NVSTRP(J)
        DO 105 II = 1, NV
C
          I = IJFRST(J) + (II-1)
C
          DO N = 1, NCONTROL
            ENV_D(1,I,N) = 0.
            ENV_D(2,I,N) = 0.
            ENV_D(3,I,N) = 0.
            ENC_D(1,I,N) = 0.
            ENC_D(2,I,N) = 0.
            ENC_D(3,I,N) = 0.
          ENDDO
C
          DO N = 1, NDESIGN
            ENV_G(1,I,N) = 0.
            ENV_G(2,I,N) = 0.
            ENV_G(3,I,N) = 0.
            ENC_G(1,I,N) = 0.
            ENC_G(2,I,N) = 0.
            ENC_G(3,I,N) = 0.
          ENDDO
C
C...Define unit vector along bound leg
          DXB = RV2(1,I)-RV1(1,I)
          DYB = RV2(2,I)-RV1(2,I)
          DZB = RV2(3,I)-RV1(3,I)
          EMAG = SQRT(DXB**2 + DYB**2 + DZB**2)
          EB(1) = DXB/EMAG
          EB(2) = DYB/EMAG
          EB(3) = DZB/EMAG
C
C...Define direction of normal vector at control point 
C   The YZ projection of the normal vector matches the camber slope
C   + section local incidence in the YZ defining plane for the section
          ANG = AINC(J) - ATAN(SLOPEC(I))
cc          IF(LDES) THEN
C--------- add design-variable contribution to angle
           DO N = 1, NDESIGN
             ANG = ANG + AINC_G(J,N)*DELDES(N)
           ENDDO
cc          ENDIF
C
          SINC = SIN(ANG)
          COSC = COS(ANG)
          EC(1) =  COSC
          EC(2) = -SINC*ES(2)
          EC(3) = -SINC*ES(3)
          DO N = 1, NDESIGN
            EC_G(1,N) = -SINC      *AINC_G(J,N)
            EC_G(2,N) = -COSC*ES(2)*AINC_G(J,N)
            EC_G(3,N) = -COSC*ES(3)*AINC_G(J,N)
          ENDDO
C
C...Normal vector is perpendicular to camberline vector and to the bound leg
          CALL CROSS(EC,EB,ECXB)
          EMAG = SQRT(ECXB(1)**2 + ECXB(2)**2 + ECXB(3)**2)
          IF(EMAG.NE.0.0) THEN
            ENC(1,I) = ECXB(1)/EMAG
            ENC(2,I) = ECXB(2)/EMAG
            ENC(3,I) = ECXB(3)/EMAG
            DO N = 1, NDESIGN
              CALL CROSS(EC_G(1,N),EB,ECXB_G)
              EMAG_G = ENC(1,I)*ECXB_G(1)
     &               + ENC(2,I)*ECXB_G(2)
     &               + ENC(3,I)*ECXB_G(3)
              ENC_G(1,I,N) = (ECXB_G(1) - ENC(1,I)*EMAG_G)/EMAG
              ENC_G(2,I,N) = (ECXB_G(2) - ENC(2,I)*EMAG_G)/EMAG
              ENC_G(3,I,N) = (ECXB_G(3) - ENC(3,I)*EMAG_G)/EMAG
            ENDDO
          ELSE
            ENC(1,I) = ES(1)
            ENC(2,I) = ES(2)
            ENC(3,I) = ES(3)
          ENDIF
C
C
C...Define direction of normal vector at vortex mid-point. 
C   The YZ projection of the normal vector matches the camber slope
C   + section local incidence in the YZ defining plane for the section
          ANG = AINC(J) - ATAN(SLOPEV(I)) 
cc          IF(LDES) THEN
C--------- add design-variable contribution to angle
           DO N = 1, NDESIGN
             ANG = ANG + AINC_G(J,N)*DELDES(N)
           ENDDO
cc          ENDIF
C
          SINC = SIN(ANG)
          COSC = COS(ANG)
          EC(1) =  COSC
          EC(2) = -SINC*ES(2)
          EC(3) = -SINC*ES(3)
          DO N = 1, NDESIGN
            EC_G(1,N) = -SINC      *AINC_G(J,N)
            EC_G(2,N) = -COSC*ES(2)*AINC_G(J,N)
            EC_G(3,N) = -COSC*ES(3)*AINC_G(J,N)
          ENDDO
C
C...Normal vector is perpendicular to camberline vector and to the bound leg
          CALL CROSS(EC,EB,ECXB)
          EMAG = SQRT(ECXB(1)**2 + ECXB(2)**2 + ECXB(3)**2)
          IF(EMAG.NE.0.0) THEN
            ENV(1,I) = ECXB(1)/EMAG
            ENV(2,I) = ECXB(2)/EMAG
            ENV(3,I) = ECXB(3)/EMAG
            DO N = 1, NDESIGN
              CALL CROSS(EC_G(1,N),EB,ECXB_G)
              EMAG_G = ENC(1,I)*ECXB_G(1)
     &               + ENC(2,I)*ECXB_G(2)
     &               + ENC(3,I)*ECXB_G(3)
              ENV_G(1,I,N) = (ECXB_G(1) - ENV(1,I)*EMAG_G)/EMAG
              ENV_G(2,I,N) = (ECXB_G(2) - ENV(2,I)*EMAG_G)/EMAG
              ENV_G(3,I,N) = (ECXB_G(3) - ENV(3,I)*EMAG_G)/EMAG
            ENDDO
          ELSE
            ENV(1,I) = ES(1)
            ENV(2,I) = ES(2)
            ENV(3,I) = ES(3)
          ENDIF
C
C
ccc       write(*,*) i, dcontrol(i,1), dcontrol(i,2)
C
C=======================================================
C-------- rotate normal vectors for control surface
          DO 100 N = 1, NCONTROL
C
C---------- skip everything if this element is unaffected by control variable N
            IF(DCONTROL(I,N).EQ.0.0) GO TO 100
C
            ANG     = DTR*DCONTROL(I,N)*DELCON(N)
            ANG_DDC = DTR*DCONTROL(I,N)
C
            COSD = COS(ANG)
            SIND = SIN(ANG)
C
C---------- EP = normal-vector component perpendicular to hinge line
            ENDOT = DOT(ENC(1,I),VHINGE(1,J,N))
            EP(1) = ENC(1,I) - ENDOT*VHINGE(1,J,N)
            EP(2) = ENC(2,I) - ENDOT*VHINGE(2,J,N)
            EP(3) = ENC(3,I) - ENDOT*VHINGE(3,J,N)
C---------- EQ = unit vector perpendicular to both EP and hinge line
            CALL CROSS(VHINGE(1,J,N),EP,EQ)
C
C---------- rotated vector would consist of sin,cos parts from EP and EQ,
C-          with hinge-parallel component ENDOT restored 
cc          ENC(1,I) = EP(1)*COSD + EQ(1)*SIND + ENDOT*VHINGE(1,J,N)
cc          ENC(2,I) = EP(2)*COSD + EQ(2)*SIND + ENDOT*VHINGE(2,J,N)
cc          ENC(3,I) = EP(3)*COSD + EQ(3)*SIND + ENDOT*VHINGE(3,J,N)
C
C---------- linearize about zero deflection (COSD=1, SIND=0)
            ENC_D(1,I,N) = ENC_D(1,I,N) + EQ(1)*ANG_DDC
            ENC_D(2,I,N) = ENC_D(2,I,N) + EQ(2)*ANG_DDC
            ENC_D(3,I,N) = ENC_D(3,I,N) + EQ(3)*ANG_DDC
C
C
C---------- repeat for ENV vector
C
C---------- EP = normal-vector component perpendicular to hinge line
            ENDOT = DOT(ENV(1,I),VHINGE(1,J,N))
            EP(1) = ENV(1,I) - ENDOT*VHINGE(1,J,N)
            EP(2) = ENV(2,I) - ENDOT*VHINGE(2,J,N)
            EP(3) = ENV(3,I) - ENDOT*VHINGE(3,J,N)
C---------- EQ = unit vector perpendicular to both EP and hinge line
            CALL CROSS(VHINGE(1,J,N),EP,EQ)
C
C---------- rotated vector would consist of sin,cos parts from EP and EQ,
C-          with hinge-parallel component ENDOT restored 
cc          ENV(1,I) = EP(1)*COSD + EQ(1)*SIND + ENDOT*VHINGE(1,J,N)
cc          ENV(2,I) = EP(2)*COSD + EQ(2)*SIND + ENDOT*VHINGE(2,J,N)
cc          ENV(3,I) = EP(3)*COSD + EQ(3)*SIND + ENDOT*VHINGE(3,J,N)
C
C---------- linearize about zero deflection (COSD=1, SIND=0)
            ENV_D(1,I,N) = ENV_D(1,I,N) + EQ(1)*ANG_DDC
            ENV_D(2,I,N) = ENV_D(2,I,N) + EQ(2)*ANG_DDC
            ENV_D(3,I,N) = ENV_D(3,I,N) + EQ(3)*ANG_DDC
 100      CONTINUE
 101      CONTINUE
C
 105    CONTINUE
  10  CONTINUE
C
      LENC = .TRUE.
C
      RETURN
      END ! ENCALC

