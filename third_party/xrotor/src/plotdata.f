C***********************************************************************
C    Module:  plotdata.f
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
      SUBROUTINE PLOT_DATA(PLTTYPE)
C----Plots data on blade
C
      INCLUDE 'XROTOR.INC'
C
      LOGICAL LERROR, LXRELIMIT, LYRELIMIT
      LOGICAL LPLT_DATA, LEGEND2D
      CHARACTER*4 OPT
      CHARACTER*6 XVAR, YVAR
      CHARACTER PLTTYPE*(*)
      CHARACTER DATALBL*20, LEGENDDATA*80
      CHARACTER*80 LINE
      CHARACTER*80 TITLE1, TITLE2
C
C
      DIMENSION RINPUT(10), XLIM(2), YLIM(2)
C
C--- Temporary data for plotting 
      DIMENSION  NPTDATA(IXP)
      DIMENSION  INFODATA(2,IXP), IFLW2DATA(0:IXP)
      DIMENSION  XDATA(IXP,IXP), YDATA(IXP,IXP)
      DIMENSION  LEGENDDATA(IXP)
C
      DATA XVAR, YVAR / 'XI', 'GAM' /
C
C
      LXRELIMIT = .TRUE.
      LYRELIMIT = .TRUE.
      XMINSPEC = 999.
      XMAXSPEC = 999.
      YMINSPEC = 999.
      YMAXSPEC = 999.
      DATALBL = PLTTYPE
C
 1    FORMAT(A)
      NDATA = 0
      TITLE1 = 'XROTOR Blade Data'
      GO TO 100
C
C--- Make the plot
 5    IF(LXRELIMIT) THEN
        XLIM(1) = -999.
        XLIM(2) = -999.
      ENDIF
      IF(LYRELIMIT) THEN
        YLIM(1) = -999.
        YLIM(2) = -999.
      ENDIF
      IF(NDATA.GT.0) THEN 
       CALL PLOTXY2(IXP,
     &              NDATA,NPTDATA,
     &              INFODATA,XDATA,YDATA,
     &              XMINSPEC,XMAXSPEC,YMINSPEC,YMAXSPEC,
     &              XVAR,YVAR,XLIM,YLIM,
     &              LEGEND2D,LEGENDDATA,
     &              TITLE1, NAME, LPLOT,
     &              PAR,CSIZE,XMARG,YMARG,NCOLOR,SCRNFR,IDEV)
       LPLOT = .TRUE.
      ENDIF
C  
C---- Display options 
 10   WRITE(*,*) ' '
      WRITE(*,20) XVAR,YVAR,LEGEND
 20   FORMAT(/' ================================================'
     &       /'   A  abscissa select   currently: ',A,
     &       /'   O  ordinate select   currently: ',A,
     &       /'   L  limits for plot',
     &       /'   Z  zoom plot with cursor',
     &       /'   R  reset plot limits',
     &       /'   AN annotate plot',
     &       /'   LG legend for plot   currently: ',L2,
     &       /'   H  hardcopy plot',
     &       /'   W  write plot data to file',
     &      //' Select option (or <return>):  ', $)
C
      READ(*,1,ERR=10) OPT
      CALL LC2UC(OPT)
C
      IF(OPT.EQ.' ') THEN
        RETURN
C
C***************************************************
C---- Make hardcopy
       ELSE IF(OPT.EQ.'H') THEN
        CALL REPLOT(IDEVRP)
        GO TO 10
C
C***************************************************
C---- Write plot data to file
       ELSE IF(OPT.EQ.'W') THEN
        IF(NDATA.GT.0) THEN
         CALL ASKS('Enter plot data save filename^',FNAME)
         OPEN(LUTEMP,FILE=FNAME,STATUS='UNKNOWN',ERR=10)
         ND = 1
         NP = NPTDATA(ND)
C--- Write out X data, Y data 
         DO I = 1, NP
           WRITE(LUTEMP,51) XDATA(I,ND),YDATA(I,ND) 
         END DO
 51      FORMAT(2(G13.6,2X))
         CLOSE(LUTEMP)
        ENDIF
        GO TO 10
C
C***************************************************
C---- Annotate plot
       ELSE IF(OPT.EQ.'AN') THEN
        CALL ANNOT(CSIZE)
        GO TO 10
C
C***************************************************
C---- Zoom plot limits
       ELSE IF(OPT.EQ.'Z') THEN
        CALL OFFSET2D(XLIM,YLIM)
        LXRELIMIT = .FALSE.
        LYRELIMIT = .FALSE.
        GO TO 5
C
C***************************************************
C---- Toggle legend flag
       ELSE IF(OPT.EQ.'LG') THEN
        LEGEND2D = .NOT.LEGEND2D
        GO TO 5
C
C***************************************************
C---- Reset plot limits
       ELSE IF(OPT.EQ.'R') THEN
        LXRELIMIT = .TRUE.
        LYRELIMIT = .TRUE.
        GO TO 5
C
C***************************************************
C---- Set plot limits
       ELSE IF(OPT.EQ.'L') THEN
        WRITE(*,*) 'Limits for plot:'
        WRITE(*,32) XLIM
 32     FORMAT('Xlimits  xmin: ',G12.6,' xmax: ',G12.6)
        READ(*,1) LINE
        IF(LINE.NE.' ') THEN 
          READ(LINE,*,ERR=10) XLIM(1), XLIM(2)
          LXRELIMIT = .FALSE.
        ENDIF
        WRITE(*,34) YLIM
 34     FORMAT('Ylimits  ymin: ',G12.6,' ymax: ',G12.6)
        READ(*,1) LINE
        IF(LINE.NE.' ') THEN 
          READ(LINE,*,ERR=10) YLIM(1), YLIM(2)
          LYRELIMIT = .FALSE.
        ENDIF
        GO TO 5
C
C***************************************************
C---- Change Abscissa 
       ELSE IF(OPT.EQ.'A') THEN
 40     WRITE(*,*) ' '
        WRITE(*,45) XVAR
 45     FORMAT(/' ================================================'
     &       /' Abscissa is: ',A,
     &       /'   XI, XW'
     &      //' Select option (or <return>):  ', $)
        READ(*,1,ERR=40) OPT
        CALL LC2UC(OPT)
        IF(OPT.EQ.' ') THEN
          GO TO 10
         ELSE IF(OPT.EQ.'XI') THEN
          XVAR = 'XI'
         ELSE IF(OPT.EQ.'XW') THEN
          XVAR = 'XW'
        ENDIF
        XMINSPEC = 999.
        XMAXSPEC = 999.
        LXRELIMIT = .TRUE.
        GO TO 100
C
C***************************************************
C---- Change Ordinate
       ELSE IF(OPT.EQ.'O') THEN
 50     WRITE(*,*) ' '
        WRITE(*,55) YVAR
 55     FORMAT(/' ================================================'
     &       /' Ordinate is: ',A,
     &       /'   CH, BE, GAM, CL, CD, RE, EFP, ',
     &       /'   Ub, VA, VT, VD, VA/V, VT/V, VD/V ',
     &       /'   VAslip, VTslip, Aslip, ',
     &       /'   Ti, Pi, Tv, Pv, Ttot, Ptot, ',
     &       /'   Xw, Vw, Tw, Pw',
     &      //' Select option (or <return>):  ', $)
        READ(*,1,ERR=50) OPT
        CALL LC2UC(OPT)
        YMINSPEC = 999.
        YMAXSPEC = 999.
        TITLE1 = 'XROTOR Blade Data'
C
        IF(OPT.EQ.' ') THEN
          GO TO 10
         ELSE IF(OPT.EQ.'CH') THEN
          YVAR = 'CH'
         ELSE IF(OPT.EQ.'BE') THEN
          YVAR = 'BE'
         ELSE IF(OPT.EQ.'GAM') THEN
          YVAR = 'GAM'
         ELSE IF(OPT.EQ.'CL') THEN
          YVAR = 'CL'
         ELSE IF(OPT.EQ.'CD') THEN
          YVAR = 'CD'
         ELSE IF(OPT.EQ.'RE') THEN
          YVAR = 'RE'
         ELSE IF(OPT.EQ.'EFP') THEN
          YVAR = 'EFFP'
C
         ELSE IF(OPT.EQ.'UB') THEN
          YVAR = 'UBOD'
         ELSE IF(OPT.EQ.'VA') THEN
          YVAR = 'VA'
         ELSE IF(OPT.EQ.'VT') THEN
          YVAR = 'VT'
         ELSE IF(OPT.EQ.'VD') THEN
          YVAR = 'VD'
         ELSE IF(OPT.EQ.'VA/V') THEN
          YVAR = 'VA/V'
         ELSE IF(OPT.EQ.'VT/V') THEN
          YVAR = 'VT/V'
         ELSE IF(OPT.EQ.'VD/V') THEN
          YVAR = 'VD/V'
         ELSE IF(OPT.EQ.'VASL') THEN
          YVAR = 'VAslip'
          TITLE1 = 'XROTOR Slipstream Data'
         ELSE IF(OPT.EQ.'VTSL') THEN
          YVAR = 'VTslip'
          TITLE1 = 'XROTOR Slipstream Data'
         ELSE IF(OPT.EQ.'ASLI') THEN
          YVAR = 'Aslip'
          TITLE1 = 'XROTOR Slipstream Data'
C
         ELSE IF(OPT.EQ.'TI') THEN
          YVAR = 'Tinv'
         ELSE IF(OPT.EQ.'PI') THEN
          YVAR = 'Pinv'
         ELSE IF(OPT.EQ.'TV') THEN
          YVAR = 'Tvis'
         ELSE IF(OPT.EQ.'PV') THEN
          YVAR = 'Pvis'
         ELSE IF(OPT.EQ.'TTOT') THEN
          YVAR = 'Ti+v'
         ELSE IF(OPT.EQ.'PTOT') THEN
          YVAR = 'Pi+v'
C--- Equivalent prop data
         ELSE IF(OPT.EQ.'XW') THEN
          YVAR = 'XW'
          TITLE1 = 'XROTOR Equivalent Prop Blade Data'
         ELSE IF(OPT.EQ.'VW') THEN
          YVAR = 'VW'
          TITLE1 = 'XROTOR Equivalent Prop Blade Data'
         ELSE IF(OPT.EQ.'TW') THEN
          YVAR = 'Twak'
          TITLE1 = 'XROTOR Equivalent Prop Blade Data'
         ELSE IF(OPT.EQ.'PW') THEN
          YVAR = 'Pwak'
          TITLE1 = 'XROTOR Equivalent Prop Blade Data'
        ENDIF
        LYRELIMIT = .TRUE.
        GO TO 100
C
      ENDIF
      GO TO 10
C
C--- Put selected data into arrays
 100  CONTINUE
      ND = 0
      NCASESX = 1
      DO NS= 1, NCASESX
        ND = ND + 1
        NP = II
        IFLW = KCASE
        INFODATA(1,ND) = IFLW
C--- Legend for each curve 
        CALL STRIP(DATALBL,NLBL)
        WRITE(LEGENDDATA(ND),101) 'Case', NS,ADV,VEL,ALT
 101    FORMAT(' ',A,' ',I3,' @ ',3(F9.3,','))
C
C
        DO I = 1, NP
C--- Install X data
          IF(XVAR.EQ.'XI') THEN
            XDATA(I,ND) = XI(I)
           ELSEIF(XVAR.EQ.'XW') THEN
            XDATA(I,ND) = XW(I)
          ENDIF
C--- Install Y data
          IF(YVAR.EQ.'CH') THEN
            YDATA(I,ND) = CH(I)
           ELSEIF(YVAR.EQ.'BE') THEN
            YDATA(I,ND) = BETA(I)
           ELSEIF(YVAR.EQ.'GAM') THEN
            YDATA(I,ND) = GAM(I)
           ELSEIF(YVAR.EQ.'CL') THEN
            YDATA(I,ND) = CL(I)
           ELSEIF(YVAR.EQ.'CD') THEN
            YDATA(I,ND) = CD(I)
           ELSEIF(YVAR.EQ.'RE') THEN
            YDATA(I,ND) = RE(I)
           ELSEIF(YVAR.EQ.'EFP') THEN
            YDATA(I,ND) = EFFP(I)
C
           ELSEIF(YVAR.EQ.'UBOD') THEN
            YDATA(I,ND) = UBODY(I)
           ELSEIF(YVAR.EQ.'VA') THEN
            YDATA(I,ND) = VIND(1,I)*VEL
           ELSEIF(YVAR.EQ.'VT') THEN
            YDATA(I,ND) = VIND(3,I)*VEL
           ELSEIF(YVAR.EQ.'VD') THEN
            VADUCT_VA = 1.0
            IF(DUCT) THEN
             VADUCT_VA = 2.0*URDUCT
            ENDIF
            YDATA(I,ND) = VIND(1,I)*VEL * (VADUCT_VA - 1.0)
           ELSEIF(YVAR.EQ.'VA/V') THEN
            YDATA(I,ND) = VIND(1,I) 
           ELSEIF(YVAR.EQ.'VT/V') THEN
            YDATA(I,ND) = VIND(3,I)
           ELSEIF(YVAR.EQ.'VD/V') THEN
            VADUCT_VA = 1.0
            IF(DUCT) THEN
             VADUCT_VA = 2.0*URDUCT
            ENDIF
            YDATA(I,ND) = VIND(1,I) * (VADUCT_VA - 1.0)
           ELSEIF(YVAR.EQ.'VAslip') THEN
            BLDS = FLOAT(NBLDS)
C------ use circumferentially averaged induced velocities 
            VT = BLDS*GAM(I)/(4.0*PI*XI(I))
            VA = VT*XI(I)/ADW
C------ include duct effect on freestream and induced axial velocity
            UDUCT     = 0.0
            VADUCT_VA = 1.0
            IF(DUCT) THEN
              UDUCT = URDUCT-1.0
              VADUCT_VA = 2.0*URDUCT
            ENDIF
            VA = VA * VADUCT_VA
            UTOT = 1.0 + UDUCT + UBODY(I)
            CALL UVADD(XI(I),WA,WT)
            CI = XI(I)/ADV - WT  -  VT
            SI = UTOT      + WA  +  VA
            YDATA(I,ND) = SI*VEL
           ELSEIF(YVAR.EQ.'VTslip') THEN
            BLDS = FLOAT(NBLDS)
            OMEG = VEL/(ADV*RAD)
            DPP = (DPII(I) + DPVI(I)) * (RHO * VEL**3 * RAD**2)
            DQQ = DPP/OMEG
            VADUCT_VA = 1.0
            IF(DUCT) THEN
             VADUCT_VA = 2.0*URDUCT
            ENDIF
            VA = VIND(1,I) * VADUCT_VA
            DQQ  = DQQ / (RHO * VEL**2 * RAD**3)
            YDATA(I,ND) = BLDS*DQQ / (2.0*PI*(1.0+VA)*((XI(I))**2))*VEL
           ELSEIF(YVAR.EQ.'Aslip') THEN
            BLDS = FLOAT(NBLDS)
C------ use circumferentially averaged induced velocities 
            VT = BLDS*GAM(I)/(4.0*PI*XI(I))
            VA = VT*XI(I)/ADW
C------ include duct effect on freestream and induced axial velocity
            UDUCT     = 0.0
            VADUCT_VA = 1.0
            IF(DUCT) THEN
              UDUCT = URDUCT-1.0
              VADUCT_VA = 2.0*URDUCT
            ENDIF
            VA = VA * VADUCT_VA
            UTOT = 1.0 + UDUCT + UBODY(I)
            CALL UVADD(XI(I),WA,WT)
            CI = XI(I)/ADV - WT  -  VT
            SI = UTOT      + WA  +  VA
            OMEG = VEL/(ADV*RAD)
            DPP = (DPII(I) + DPVI(I)) * (RHO * VEL**3 * RAD**2)
            DQQ = DPP/OMEG
            DQQ  = DQQ / (RHO * VEL**2 * RAD**3)
            VTA = BLDS*DQQ / (2.0*PI*(1.0+VA)*((XI(I))**2))
            YDATA(I,ND) = ATAN2(VTA,SI) * 180.0/PI
C
           ELSEIF(YVAR.EQ.'Tinv') THEN
            YDATA(I,ND) = DTII(I)*RHO * VEL**2 * RAD**2
           ELSEIF(YVAR.EQ.'Pinv') THEN
            YDATA(I,ND) = DPII(I)*RHO * VEL**3 * RAD**2
           ELSEIF(YVAR.EQ.'Tvis') THEN
            YDATA(I,ND) = DTVI(I)*RHO * VEL**2 * RAD**2
           ELSEIF(YVAR.EQ.'Pvis') THEN
            YDATA(I,ND) = DPVI(I)*RHO * VEL**3 * RAD**2
           ELSEIF(YVAR.EQ.'Ti+v') THEN
            YDATA(I,ND) = (DTII(I)+DTVI(I))*RHO * VEL**2 * RAD**2
           ELSEIF(YVAR.EQ.'Pi+v') THEN
            YDATA(I,ND) = (DPII(I)+DPVI(I))*RHO * VEL**3 * RAD**2
C--- Equivalent prop data
           ELSEIF(YVAR.EQ.'XW') THEN
            YDATA(I,ND) = XW(I)
           ELSEIF(YVAR.EQ.'VW') THEN
            YDATA(I,ND) = VWAK(I)*VEL
           ELSEIF(YVAR.EQ.'Twak') THEN
            YDATA(I,ND) = DTWI(I)*RHO * VEL**2 * RAD**2
           ELSEIF(YVAR.EQ.'Pwak') THEN
            YDATA(I,ND) = DPWI(I)*RHO * VEL**3 * RAD**2
          ENDIF
        END DO
        NPTDATA(ND) = NP
C
      END DO
      NDATA = ND
      GO TO 5
C    
      END


      SUBROUTINE PLOTXY2(NDIM,
     &                   NDATA,NPTDATA,
     &                   INFODATA,XDATA,YDATA,
     &                   XMINSPEC,XMAXSPEC,YMINSPEC,YMAXSPEC,
     &                   XVAR,YVAR,XLIM,YLIM,
     &                   LPLTLEGEND,LEGENDLABEL,
     &                   TITLE1, TITLE2, LPLOT,
     &                   AR,CH,XMARG,YMARG,NCOLOR,SCRNFR,IDEV)
C
C--- Plot XY data in multiple data arrays
C
      PARAMETER ( ITMPX=500 )
C
      CHARACTER XVAR*(*), YVAR*(*)
      CHARACTER LINELABEL*80, LEGENDLABEL*(*)
      CHARACTER TITLE1*(*), TITLE2*(*)
      LOGICAL   LGRID, LPLTLEGEND, LPLOT
C
      DIMENSION  NPTDATA(*)
      DIMENSION  INFODATA(2,*)
      DIMENSION  XDATA(NDIM,*), YDATA(NDIM,*)
      DIMENSION  LEGENDLABEL(*)

      DIMENSION  XLIM(2), YLIM(2)
C
      DIMENSION  XX(ITMPX), YY(ITMPX), P1(3)
      DIMENSION  XCMX(ITMPX), XCMN(ITMPX), XCAV(ITMPX)
      DIMENSION  YCMX(ITMPX), YCMN(ITMPX), YCAV(ITMPX)
      DIMENSION  IFLW(ITMPX)
C
      COMMON / PLT2DATA / XOF2D, YOF2D, XSF2D, YSF2D
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
C--- Scaling functions to plot coordinates 
      XXMOD(XXX) = (XXX - XOF2D)*XSF2D
      YYMOD(YYY) = (YYY - YOF2D)*YSF2D
C
C
      IF(NDATA.LE.0) RETURN
C
C---- Character and symbol sizing
      CH2 = 0.7*CH
      CH3 = 0.6*CH2
      SH  = 0.6*CH2
C---- local plot aspect ratio
      PAR = 0.90*AR
C---- number of grid intervals per axis annotation interval
      NGR = 2
C---- origin location / size
      XORG = 0.125
      YORG = 0.10
C---- Defaults
      LGRID = .TRUE.
      SIZE = 9.0
C
C
C***************************************************
C---- Plot the input data arrays
C
C--- Check for plot limits on selected data
        XPAV = -1.0E10
        XPMX = -1.0E10
        XPMN =  1.0E10
        YPAV = -1.0E10
        YPMX = -1.0E10
        YPMN =  1.0E10
        NSEL = 0
        NCHLEGEND = 0
        DO NC = 1, NDATA
           NSEL = NSEL + 1
C
           LINELABEL = LEGENDLABEL(NC)
           CALL STRIP(LINELABEL,NLBL)
           NCHLEGEND = MAX(NCHLEGEND,NLBL)
C--- Get case #s
           IFLW(NSEL) = INFODATA(1,NC)
           NP = NPTDATA(NC)
           XAV =  0.0
           XMX = -1.0E10
           XMN =  1.0E10
           YAV =  0.0
           YMX = -1.0E10
           YMN =  1.0E10
           DO I = 1, NP
             XMX = MAX(XMX,XDATA(I,NC))
             XMN = MIN(XMN,XDATA(I,NC))
             XAV = XAV + XDATA(I,NC)
             NAV = NAV + 1
             YMX = MAX(YMX,YDATA(I,NC))
             YMN = MIN(YMN,YDATA(I,NC))
             YAV = YAV + YDATA(I,NC)
           END DO
           XAV = XAV / FLOAT(NP)
           YAV = YAV / FLOAT(NP)
           XCMX(NC) = XMX   
           XCMN(NC) = XMN   
           YCMX(NC) = YMX   
           YCMN(NC) = YMN   
           XCAV(NC) = XAV   
           YCAV(NC) = YAV   
C
           XPMX = MAX(XPMX,XMX)
           XPMN = MIN(XPMN,XMN)
           YPMX = MAX(YPMX,YMX)
           YPMN = MIN(YPMN,YMN)
           XPAV = MAX(XPAV,XAV)
           YPAV = MAX(YPAV,YAV)
        END DO
C
        XMX = XPMX
        XMN = XPMN
        YMX = YPMX
        YMN = YPMN
C
        IF(NSEL.LE.0) THEN
         WRITE(*,*) 'No plot data selected!'
         RETURN
        ENDIF
C
C--- Adjustments to min/max values for specific quantities
        IF(YVAR.EQ.'CP') THEN  
          YMN = 0.0
          YMX = YPAV
        ENDIF
C
C--- Adjustments to min/max values for specific quantities
        IF(XMINSPEC.NE.999.)  XMN = XMINSPEC
        IF(XMAXSPEC.NE.999.)  XMX = XMAXSPEC
        IF(YMINSPEC.NE.999.)  YMN = YMINSPEC
        IF(YMAXSPEC.NE.999.)  YMX = YMAXSPEC
C
        IF(YMX.EQ.YMN) THEN
          YMX = YMN + 1.0
        ENDIF
C
C--- Check limits imposed (-999. is autorange)
        IF(XLIM(1).NE.-999.) XMN = XLIM(1)
        IF(XLIM(2).NE.-999.) XMX = XLIM(2)
        IF(YLIM(1).NE.-999.) YMN = YLIM(1)
        IF(YLIM(2).NE.-999.) YMX = YLIM(2)
C
        IF(ABS(XMX-XMN).LT.1.0E-10) THEN
         XMX = XMN + 1.0
        ENDIF
        IF(ABS(YMX-YMN).LT.1.0E-10) THEN
         YMX = YMN + 1.0
        ENDIF
C
        IF(XMX.LT.XMN) THEN
         TMP = XMN
         XMN = XMX
         XMX = TMP
         IF(XMX.EQ.XMN) XMX = XMN + 1.0
        ENDIF
        IF(YMX.LT.YMN) THEN
         TMP = YMN
         YMN = YMX
         YMX = TMP
         IF(YMX.EQ.YMN) YMX = YMN + 1.0
        ENDIF
C
C--- Scale the axes to plot
        CALL AXISADJ(XMN,XMX,XSPAN,DELTAX,NXTICS)
        CALL AXISADJ(YMN,YMX,YSPAN,DELTAY,NYTICS)
C
        XLIM(1) = XMN
        XLIM(2) = XMX
        YLIM(1) = YMN
        YLIM(2) = YMX
C
C---- set plot offsets and scaling factors
        XAXISLEN = 0.9
        YAXISLEN = 0.9*PAR
C
        XOF2D  = XMN
        XSF2D  = XAXISLEN / XSPAN
        YOF2D  = YMN
        YSF2D  = YAXISLEN / YSPAN
C
        XAXLEN = XSPAN*XSF2D
        YAXLEN = YSPAN*YSF2D
C
        XLEGND   = 1.10*XAXLEN - FLOAT(NCHLEGEND)*CH3
        YLEGND   = 1.0*YAXLEN
        IF(YVAR.EQ.'D') THEN
          XLEGND = 0.1*XAXLEN
        ENDIF
C
        Y0     = YMN
        YSTRT  = YMN
        YLAB   = YMX
C
        DXANN  = DELTAX*XSF2D
        DYANN  = DELTAY*YSF2D
C
C
C------ open window and plot axes for current scale/offset
        CALL GETCOLOR(ICOL0)
        CALL NEWCOLORNAME('BLACK')
C
        IF (LPLOT) CALL PLEND
        CALL PLOPEN(SCRNFR,0,IDEV)
        LPLOT = .TRUE. 
        CALL PLOTABS(XORG*SIZE+XMARG,YORG*SIZE+YMARG,-3)
        CALL NEWPEN(2)
C
        CALL NEWFACTOR(SIZE)
C
C--- Title
        XPLT = 0.0
        YPLT = YAXLEN + 3.0*CH2
        CALL NEWPEN(4)
        CALL PLCHAR(XPLT,YPLT,1.1*CH2,TITLE1,0.0,-1)
        CALL NEWPEN(3)
        YPLT = YAXLEN + 1.0*CH2
        CALL PLCHAR(XPLT,YPLT,1.0*CH2,TITLE2,0.0,-1)
C
C--- X axis with first annotation suppressed
        CALL XAXIS2(XXMOD(XMN),YYMOD(Y0),
     &             XAXLEN,DXANN,XMN,DELTAX,1,CH2,-2)
        CALL STRIP(XVAR,NXCH)
        XPLT = XXMOD(XMX)-1.5*DXANN-0.5*CH*NXCH
        YPLT = YYMOD(Y0)-0.5*FLOAT(LEN(XVAR))*CH
        CALL PLCHAR(XPLT,YPLT,1.1*CH,XVAR,0.0,NXCH)
C
C--- Y axis
        CALL YAXIS(XXMOD(XMN),YYMOD(YSTRT),
     &             YAXLEN,DYANN,YSTRT,DELTAY,CH2,-2)
        CALL STRIP(YVAR,NYCH)
        XPLT = XXMOD(XMN)  - CH*FLOAT(NYCH+1)
        YPLT = YYMOD(YLAB) - 0.5*CH
        IF(NYTICS.LE.3) THEN
           YPLT = YPLT - 0.5*DYANN
          ELSE
           YPLT = YPLT - 1.5*DYANN
        ENDIF
        CALL PLCHAR(XPLT,YPLT,1.1*CH,YVAR,0.0,NYCH)
C
C--- Background grid
        IF(LGRID) THEN
         NXGR = NGR * INT(XAXISLEN/DXANN + 0.001)
         NYGR = NGR * INT(YAXISLEN/DYANN + 0.001)
         DXG = DXANN / FLOAT(NGR)
         DYG = DYANN / FLOAT(NGR)
         CALL NEWPEN(1)
         CALL PLGRID(0.0,0.0, NXGR,DXG, NYGR,DYG, LMASK2 )
        ENDIF
C
C--- Plot the data as line segments
        NCC = 0
        DO NC = 1, NDATA
          IF(NPTDATA(NC).GT.0) THEN
C
            NCC = NCC + 1
            NCMOD = NCOLOR/10
            IC = 10*MOD(NCC-1,NCMOD) + 1
            CALL NEWCOLOR(-IC)
C
            NP = NPTDATA(NC)
            NPC = NP/2
            ID = INFODATA(1,NC)


              CHORD = 1.0
              XLE   = 0.0
C
C--- Plot the point pairs on the stored line segments
            DO I = 1, NP-1
               X1 = (XDATA(I,  NC)-XLE)/CHORD
               X2 = (XDATA(I+1,NC)-XLE)/CHORD
               XX1 = XXMOD(X1)
               XX2 = XXMOD(X2)
               YY1 = YYMOD(YDATA(I  ,NC))
               YY2 = YYMOD(YDATA(I+1,NC))
C 
               CALL PLOT(XX1,YY1,3)
               CALL PLOT(XX2,YY2,2)
C 
               CALL PLSYMB(XX1,YY1,SH,NC,0.0,0)
               CALL PLSYMB(XX2,YY2,SH,NC,0.0,0)
               IF(I.EQ.NPC) THEN
                CALL PLNUMB(XX2+CH2,YY2+CH2,CH2,FLOAT(NC),0.0,-1)
               ENDIF
            END DO   
C
C--- Legend for curves
           IF(LPLTLEGEND) THEN
            LINELABEL = LEGENDLABEL(NC)
            CALL STRIP(LINELABEL,NLBL)
            XPLT = XLEGND
            YPLT = YLEGND - FLOAT(NCC-1)*2.0*CH3 - 1.8*CH3
            CALL PLSYMB(XPLT,YPLT+0.5*SH,SH,NC,0.0,0)
            CALL NEWCOLORNAME('BLACK')
            CALL PLCHAR(999.,999.,CH3,'  ',0.0,2)
            CALL PLCHAR(999.,999.,CH3,LINELABEL,0.0,NLBL)
           ENDIF
C
          ENDIF
C
        END DO
C
C
        CALL PLFLUSH
C
      CALL NEWCOLOR(ICOL0)
      RETURN
      END
  


      SUBROUTINE OFFSET2D(XLIM,YLIM)
C---- Get zoom box from user mouse selection on 2D plot 
C
      DIMENSION XLIM(2), YLIM(2)
      CHARACTER*1 CHKEY
      COMMON / PLT2DATA / XOF2D, YOF2D, XSF2D, YSF2D
C
      SH = 2.0
C
      WRITE(*,*)
      WRITE(*,*) 'Mark off corners of blowup area'
      WRITE(*,*) '(2 spaces default to current area)'       
      CALL GETCURSORXY(XX1,YY1,CHKEY)
      CALL PLSYMB(XX1,YY1,SH,3,0.0,0)
      CALL GETCURSORXY(XX2,YY2,CHKEY)
      CALL PLSYMB(XX2,YY2,SH,3,0.0,0)
      IF(ABS(XX1-XX2).LT.0.01 .AND. ABS(YY1-YY2).LT.0.01) RETURN      
C
      XOF = MIN(XX1,XX2)/XSF2D + XOF2D
      YOF = MIN(YY1,YY2)/YSF2D + YOF2D
      XDIF = ABS(XX2 - XX1)/XSF2D
      YDIF = ABS(YY2 - YY1)/YSF2D   
      IF(XDIF.NE.0.0) THEN
        XLIM(1) = XOF
        XLIM(2) = XOF + XDIF
      ENDIF
      IF(YDIF.NE.0.0) THEN
        YLIM(1) = YOF
        YLIM(2) = YOF + YDIF
      ENDIF
C
      RETURN        
      END 




      SUBROUTINE XAXIS2(X1,Y1,XAXT,DXANN,FANN,DANN,IFLAG,CHT,NDIG)
C
C---XAXIS2 differs from libPlt XAXIS by having a flag to suppress both
C   end annotations rather than the zero annotation.
C.......................................................
C     X1,Y1  starting point of x axis
C     XAXT   length of x axis 
C     DXANN  distance between annotations
C     FANN   first annotation value
C     DANN   delta annotation value
C     IFLAG  flag to suppress end annotations 
C            = 0    all annotations
C            = 1    suppress first annotation
C            = 2    suppress last  annotation
C            = 3    suppress first and last annotations
C     CHT    character height   ( - = annotation above axis)
C     NDIG   number of digits to right of decimal point
C            = -1   no decimal point
C            = -2   number of digits determined internally
C.......................................................
C
      XAX = ABS(XAXT)
      IF(XAX.LE.0) RETURN
      CH = ABS(CHT)
C
C--- determine # of digits to use for annotations
      IF(NDIG.LE.-2) THEN
        ND = 1 - MAX( 0 , INT(LOG10(DANN)) )
        IF(DANN*10**ND - AINT(DANN*10**ND+0.01) .GT. 0.01) ND = ND + 1
        IF(DANN*10**ND - AINT(DANN*10**ND+0.01) .GT. 0.01) ND = ND + 1
      ELSE
        ND = NDIG
      ENDIF
      NDG = MAX(-1,ND)
ccc      write(*,*) 'xaxis2 dann,nd,ndig,ndg ',dann,nd,ndig,ndg
C
C---- x-axis
      CALL PLOT(X1,Y1,3)
      CALL PLOT(X1+XAX,Y1,2)
      NANN = 1 + IFIX(XAX/DXANN + 0.1)
ccc      write(*,*) 'nann ',nann
C
C---- annotate x-axis
      DO 10 NT=1, NANN
        XT = X1 + DXANN*FLOAT(NT-1)
C---- skip annotation for first or last annotation position as given by IFLG
        IF(MOD(IFLAG,2).EQ.1 .AND. NT.EQ.1)    GO TO 10
        IF(IFLAG.GT.1        .AND. NT.EQ.NANN) GO TO 10
C
        CALL PLOT(XT,Y1-0.2*CH,3)
        CALL PLOT(XT,Y1+0.2*CH,2)
        RN = FANN + DANN*FLOAT(NT-1)
        GRN = 0.
        IF(RN.NE.0.0) GRN = ALOG10(ABS(RN)+0.5/10.0**ND)
        GRN = MAX(GRN,0.0)
        NABC = INT(GRN) + 2 + ND
        WIDTH = 0.95*CH*FLOAT(NABC)
        IF(RN.LT.0.0) WIDTH = WIDTH + CH
        XNUM = XT - 0.5*WIDTH
        YNUM = Y1 - 2.1*CH
        IF(CHT.LT.0.0) YNUM = Y1 + 0.9*CH
C
        CALL PLNUMB(XNUM,YNUM,CH,RN,0.0,NDG)
   10 CONTINUE
C
      RETURN
      END ! XAXIS2
 






























