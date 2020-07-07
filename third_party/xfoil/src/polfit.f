C***********************************************************************
C    Module:  polplt.f
C 
C    Copyright (C) 2000 Mark Drela 
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
C
      SUBROUTINE POLFIT(NAX,NPOL,NA,CPOL,
     &                  REYN,MACH,ACRIT, NAME ,ICOL,ILIN,
     &                  IMATYP,IRETYP,
     &                  PLOTAR, XCD,XAL,XOC, CH,CH2, CLEXP,
     &                  CPOLPLF, CCLEN,NCLEN )
C----------------------------------------------------------------
C     Generates polar plot
C----------------------------------------------------------------
      INCLUDE 'PINDEX.INC'
C
      INTEGER NA(NPOL), 
     &        ICOL(NPOL), ILIN(NPOL),
     &        IMATYP(NPOL),IRETYP(NPOL)
      REAL CPOL(NAX,IPTOT,NPOL)
      REAL CPOLPLF(3,*)
      REAL REYN(NPOL), MACH(NPOL), ACRIT(NPOL)
C----------------------------------------------------------------
      CHARACTER*1 CC
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      CALL GETVAR(NPOL,NAME,REYN,MACH,ACRIT,
     &            NAMVAR,REYVAR,MACVAR,ACRVAR)
C
C---- polar and data-symbol pen width
      IPEN = 5
C
C---- unpack plot limit array
      CLMIN = CPOLPLF(1,ICL)
      CLMAX = CPOLPLF(2,ICL)
      CLDEL = CPOLPLF(3,ICL)
C                 
      CDMIN = CPOLPLF(1,ICD)
      CDMAX = CPOLPLF(2,ICD)
      CDDEL = CPOLPLF(3,ICD)
C                 
      CMMIN = CPOLPLF(1,ICM)
      CMMAX = CPOLPLF(2,ICM)
      CMDEL = CPOLPLF(3,ICM)
C                 
      ALMIN = CPOLPLF(1,IAL)
      ALMAX = CPOLPLF(2,IAL)
      ALDEL = CPOLPLF(3,IAL)
C

c      WRITE(*,*) CPOLPLF(1,ICL),CPOLPLF(2,ICL),CPOLPLF(3,ICL)
c      WRITE(*,*) CPOLPLF(1,ICD),CPOLPLF(2,ICD),CPOLPLF(3,ICD)
c      WRITE(*,*) CPOLPLF(1,ICM),CPOLPLF(2,ICM),CPOLPLF(3,ICM)
c      WRITE(*,*) CPOLPLF(1,IAL),CPOLPLF(2,IAL),CPOLPLF(3,IAL)


C---- Get scale factor and set scale factor to 0.9 of current to fit plots
      CALL GETFACTORS(XSZ,YSZ)
      CALL NEWFACTORS(0.9*XSZ,0.9*YSZ)
C
C---- Set sane scale factors for axes
      CLWT = 1.0
      CDWT = 1.0
      CMWT = 1.0
      ALWT = 1.0
C
      CLRANGE = CLMAX-CLMIN
      IF(CLRANGE.NE.0.0) THEN
        CLWT = PLOTAR / CLRANGE
      ENDIF
C
      IF(CDMAX.NE.0.0) THEN
        CDWT = XCD/CDMAX
      ENDIF
C
C---- CM range is whole multiple of CLDEL just larger than 0.5*CLMAX
      CLMX = CLDEL * AINT( 0.5*ABS(CLMAX)/CLDEL + 0.51 )
      CMMX = MAX(ABS(CMMIN),ABS(CMMAX))
      IF(CMMX.NE.0.0) THEN
        CMWT = CLWT*CLMX/CMMX
      ENDIF
C
      ALRANGE = ALMAX-ALMIN
      IF(ALRANGE.NE.0.0) THEN
        ALWT = XAL / ALRANGE
      ENDIF
C
C
C---- number of text lines to be plotted in left upper legend in CL-CD plot
      LINBOX = NDAT
      IF(LEGND.AND. NPOL.GT.1) LINBOX = LINBOX + NPOL + 1
      DYBOX = CH2*(2.0*FLOAT(LINBOX) + 1.0)
C---- allow # CH2 character string width in label box
      NCHBOX = 18
      DXBOX = FLOAT(NCHBOX)*CH2
C

C---- set default color index
      CALL GETCOLOR(ICOL0)
C---- reorigin for CDMIN,CLMIN
      CALL PLOT(-CDWT*CDMIN,-CLWT*CLMIN,-3)
C
C---- put Polar labels above plots 
C     Labels contain: Title
C           airfoils: Name, Mach, Re, and Ncrit
C
      XPLT0 = CDWT*CDMIN
      YPLT0 = CLWT*CLMAX
      CALL POLLAB(NPOL, NAME ,ICOL,
     &           IMATYP, IRETYP,
     &           MACH, REYN, ACRIT,
     &           TITLE,
     &           XPLT0,YPLT0, PLOTAR, CH,CH2, 
     &           LLIST, CCLEN,NCLEN )
C
      CALL NEWCOLOR(ICOL0)
C
C
C--- CL-CD plot
C==================================================================
      IF(XCD.EQ.0.0) GO TO 100
C
C---- CL axis for CL-CD polar
      CALL NEWPEN(2)
      NDIG = NDIGITS(CLDEL)
      CALL YAXIS(CDWT*CDMIN,CLWT*CLMIN,PLOTAR,CLWT*CLDEL,
     &           CLMIN,CLDEL,CH2,NDIG)
C
      CALL NEWPEN(3)
      IF(NCLEN.GT.0) THEN
       XPLT = CDWT* CDMIN            - 3.0*CH - FLOAT(NCLEN)*1.2*CH
       YPLT = CLWT*(CLMAX-1.5*CLDEL) - 0.5*CH
       CALL PLCHAR(XPLT,YPLT,1.2*CH,'('  ,0.0,1)
       CALL PLCHAR(999.,YPLT,1.2*CH,CCLEN,0.0,NCLEN)
       CALL PLCHAR(999.,YPLT,1.2*CH,')'  ,0.0,1)
      ENDIF
C
      XPLT = CDWT* CDMIN            - 3.2*CH
      YPLT = CLWT*(CLMAX-0.5*CLDEL) - 0.6*CH
      IF(NCLEN.GT.0) THEN
      CALL PLCHAR(XPLT-1.1*CH,YPLT       ,1.1*CH,CC ,0.0,1)
      ENDIF
      CALL PLCHAR(XPLT       ,YPLT       ,1.4*CH,'C',0.0,1)
      CALL PLCHAR(XPLT+1.2*CH,YPLT-0.4*CH,0.9*CH,'L',0.0,1)
C
      IF(ABS(CLEXP-1.0) .GT. 0.001)
     & CALL PLNUMB(XPLT+1.05*CH,YPLT+1.3*CH,0.70*CH,CLEXP,0.0,1)
C
C---- CD axis for CL-CD polar
      CALL NEWPEN(2)
      CALL XAXIS(CDWT*CDMIN,CLWT*CLMIN,-XCD,CDWT*CDDEL,
     &           10000.*CDMIN,10000.*CDDEL,CH2,-1)
C
      CALL NEWPEN(3)
      NXL = INT((CDMAX-CDMIN)/CDDEL + 0.5)
      XPLT = CDWT*(CDMAX - (FLOAT((NXL+1)/2) - 0.5)*CDDEL) - 4.5*CH2
      YPLT = CLWT* CLMIN - 4.8*CH2
      CALL PLCHAR(XPLT       ,YPLT       ,1.4*CH,'10'    ,0.0,2)
      CALL PLMATH(XPLT       ,YPLT       ,1.4*CH,'  4'   ,0.0,3)
      CALL PLMATH(XPLT+3.9*CH,YPLT       ,1.0*CH,   '#'  ,0.0,1)
      IF(NCLEN.GT.0) THEN
      CALL PLCHAR(XPLT+4.9*CH,YPLT       ,1.1*CH,     CC ,0.0,1)
      ENDIF
      CALL PLCHAR(XPLT+6.0*CH,YPLT       ,1.4*CH,     'C',0.0,1)
      CALL PLCHAR(XPLT+7.2*CH,YPLT-0.4*CH,0.9*CH,     'D',0.0,1)
C
C--- Put legend data in legend box in upper left of CL/CD plot
      IF(LEGND) THEN
C
      YLINE = CLWT*CLMAX - 2.0*CH2
      CALL NEWPEN(3)
C
      IF(NAMVAR) THEN
       XPLT = CDWT*CDMIN + 6.0*CH2
       YPLT = YLINE
       CALL PLCHAR(XPLT    ,YPLT,    CH2,'Airfoil',0.0,7)
       YLINE = YLINE - 2.25*CH2
      ENDIF
C
      IF(REYVAR) THEN
       XPLT = CDWT*CDMIN + 7.5*CH2
       YPLT = YLINE
       ITYP = IRETYP(1)
       IF(ITYP.EQ.1) THEN
        CALL PLCHAR(XPLT        ,YPLT,    CH2,'Re'  ,0.0,2)
       ELSE IF(ITYP.EQ.2) THEN
        CALL PLMATH(XPLT-1.0*CH2,YPLT,    CH2,'  R  ',0.0,5)
        CALL PLCHAR(XPLT-1.0*CH2,YPLT,    CH2,'Re C' ,0.0,4)
        CALL PLCHAR(999.        ,999.,0.7*CH2,    'L',0.0,1)
       ELSE IF(ITYP.EQ.3) THEN
        CALL PLMATH(XPLT-1.0*CH2,YPLT,    CH2,'  #  ',0.0,5)
        CALL PLCHAR(XPLT-1.0*CH2,YPLT,    CH2,'Re C' ,0.0,4)
        CALL PLCHAR(999.        ,999.,0.7*CH2,    'L',0.0,1)
       ENDIF
       YLINE = YLINE - 2.25*CH2
      ENDIF
C
      IF(ACRVAR) THEN
       XPLT = CDWT*CDMIN + 8.0*CH2
       YPLT = YLINE
       CALL PLCHAR(XPLT,YPLT,    CH2,'N'   ,0.0,1)
       CALL PLCHAR(999.,999.,0.7*CH2,'crit',0.0,4)
       YLINE = YLINE - 2.25*CH2
      ENDIF
C
      IF(MACVAR) THEN
       XPLT = CDWT*CDMIN + 7.5*CH2
       YPLT = YLINE
       ITYP = IMATYP(1)
       IF(ITYP.EQ.1) THEN
        CALL PLCHAR(XPLT        ,YPLT,    CH2,'Ma'  ,0.0,2)
       ELSE IF(ITYP.EQ.2) THEN
        CALL PLMATH(XPLT-1.0*CH2,YPLT,    CH2,'  R  ',0.0,5)
        CALL PLCHAR(XPLT-1.0*CH2,YPLT,    CH2,'Ma C' ,0.0,4)
        CALL PLCHAR(999.        ,999.,0.7*CH2,    'L',0.0,1)
       ELSE IF(ITYP.EQ.3) THEN
        CALL PLMATH(XPLT-1.0*CH2,YPLT,    CH2,'  #  ',0.0,5)
        CALL PLCHAR(XPLT-1.0*CH2,YPLT,    CH2,'Ma C' ,0.0,4)
        CALL PLCHAR(999.        ,999.,0.7*CH2,    'L',0.0,1)
       ENDIF
       YLINE = YLINE - 2.25*CH2
      ENDIF
C
      ENDIF
C
C---- plot CL-CD polar(s)
      DO IP=1, NPOL
        CALL NEWCOLOR(ICOL(IP))
        CALL NEWPEN(IPEN)
        CALL XYLINE(NA(IP),CPOL(1,ICD,IP),CPOL(1,ICL,IP),
     &              0.,CDWT,0.,CLWT,ILIN(IP))
        IF(LCDW)
     &  CALL XYLINE(NA(IP),CPOL(1,ICW,IP),CPOL(1,ICL,IP),
     &              0.,CDWT,0.,CLWT,ILIN(IP))
      END DO
C
C---- label each polar with legend
      IF(LEGND .AND. (NAMVAR .OR. REYVAR .OR. ACRVAR .OR. MACVAR)) THEN
       DO IP=1, NPOL
         CALL NEWCOLOR(ICOL(IP))
         XLIN(1) =     CH2
         XLIN(2) = 3.0*CH2
         XLIN(3) = 6.0*CH2
         YLIN(1) = YLINE + 0.5*CH2
         YLIN(2) = YLINE + 0.5*CH2
         YLIN(3) = YLINE + 0.5*CH2
         CALL NEWPEN(IPEN)
         CALL XYLINE(3,XLIN,YLIN,0.0,1.0,0.0,1.0,ILIN(IP))
         CALL NEWPEN(2)
         XPT = CDWT*CDMIN + 7.5*CH2
         IF(NAMVAR) CALL PLCHAR(XPT,YLINE,.8*CH2,NAME(IP) ,0.,14)
         IF(REYVAR) CALL PLNUMB(XPT,YLINE,.8*CH2,REYN(IP) ,0.,-1)
         IF(ACRVAR) CALL PLNUMB(XPT,YLINE,.8*CH2,ACRIT(IP),0., 3)
         IF(MACVAR) CALL PLNUMB(XPT,YLINE,.8*CH2,MACH(IP) ,0., 3)
         YLINE = YLINE - 2.0*CH2
       END DO
       YLINE = YLINE - 0.5*CH2
C
      ENDIF
C
C
C---- plot CL-CD reference data
      DO ID=1, NDAT
        IF(NF(1,ID).NE.0) THEN
         CALL NEWPEN(IFPEN)
         CALL NEWCOLOR(IFCOL(ID))
         CALL XYSYMB(NF(1,ID),XYREF(1,1,1,ID),XYREF(1,2,1,ID),
     &               0.0,CDWT,0.0,CLWT,SH,IFSYM(ID))
         XPLT = CDWT*CDMIN + 1.5*CH2
         YPLT = YLINE + 0.5*CH2
         CALL PLSYMB(XPLT,YPLT,SH,ID,0.0,0)
         XPLT = CDWT*CDMIN + 3.0*CH2
         CALL NEWPEN(2)
         LABLEN = LEN(LABREF(ID))
         CALL PLCHAR(XPLT,YLINE,0.8*CH2,LABREF(ID),0.0,LABLEN)
         YLINE = YLINE - 2.0*CH2
        ENDIF
      END DO
      CALL NEWCOLOR(ICOL0)
C
C----- coarse grid lines
       CALL NEWPEN(1)
       DXG = CDWT*CDDEL
       DYG = CLWT*CLDEL
C----- check for legend box at top left of CL-CD grid area
       NXGBOX = INT( DXBOX/(DXG/5.0) ) + 1
       NYGBOX = INT( DYBOX/(DYG/5.0) ) + 1
       IF (LINBOX.EQ.0) THEN
         NXGBOX = 0
         NYGBOX = 0
       ENDIF
       DXGBOX = (DXG/5.0) * FLOAT(NXGBOX)
       DYGBOX = (DYG/5.0) * FLOAT(NYGBOX)
C
       Y0 = CLWT*CLMIN
       NXG = INT( XCD/(CDWT*CDDEL)    + 0.01 )
       NYG = INT( (CLMAX-CLMIN)/CLDEL + 0.01 )
C
C----- plot vertical coarse grid lines around label box
       DO K=0, NXG
        DXL = CDWT*CDDEL*FLOAT(K)
        XL = CDWT*CDMIN + DXL
        CALL PLOT(XL,Y0,3)
        IF(DXL-DXGBOX.GT. -0.001*DXGBOX) THEN
          CALL PLOT(XL, Y0 + DYG*FLOAT(NYG)       , 2)
         ELSE
          CALL PLOT(XL, Y0 + DYG*FLOAT(NYG)-DYGBOX, 2)
        ENDIF
       END DO
C
C----- plot horizontal coarse grid lines around label box
       Y0 = CLWT*CLMAX
       CALL PLOT(CDWT*CDMIN, Y0, 3)
       CALL PLOT(CDWT*CDMAX, Y0, 2)
       DO K=1, NYG
         DYL = CLWT*CLDEL*FLOAT(K)
         YL = Y0 - DYL
         X0 = CDWT*CDMAX
         IF(DYL-DYGBOX.GT.-0.001*DYGBOX) THEN
           CALL PLOT(CDWT*CDMIN, YL, 3)
          ELSE
           CALL PLOT(CDWT*CDMIN+DXGBOX, YL, 3)
         ENDIF
         CALL PLOT(CDWT*CDMAX, YL, 2)
       END DO
C
C---- plot edges of label box
       Y0 = CLWT*CLMAX-DYGBOX
       CALL PLOT(CDWT*CDMIN, Y0, 3)
       CALL PLOT(CDWT*CDMIN+DXGBOX, Y0, 2)
       CALL PLOT(CDWT*CDMIN+DXGBOX, Y0+DYGBOX, 2)
C
C----- fine grid
      IF(LGRID) THEN
       CALL NEWPEN(1)
       DXG = CDWT*CDDEL / 5.0
       DYG = CLWT*CLDEL / 5.0
       X0 = CDWT*CDMIN
       Y0 = CLWT*CLMIN
C---- plot fine grid under the label box, if present
       NXGF = NXGBOX
       NYGF = 5*NYG - NYGBOX
       IF(NXGF.GT.0) CALL PLGRID(X0,Y0, NXGF,DXG, NYGF,DYG, LMASK2 )
C---- plot fine grid right of the label box
       X0 = X0 + DXG*FLOAT(NXGF)
       NXGF = 5*NXG - NXGF
       NYGF = 5*NYG
       CALL PLGRID(X0,Y0, NXGF,DXG, NYGF,DYG, LMASK2 )
      ENDIF
C
C--- CL-alfa plot
C==================================================================
C---- re-origin for CL-a plot
      CALL PLOT(CDWT*CDMAX + 0.05 - ALWT*ALMIN,0.0,-3)
C
 100  CONTINUE
      IF(XAL.EQ.0.0) GO TO 200
C
C---- CL axis for CL-a plot
      CALL NEWPEN(2)
      CALL YAXIS(0.0,CLWT*CLMIN,-PLOTAR,CLWT*CLDEL,CLMIN,CLDEL,-CH2,1)
C
      CALL NEWPEN(3)
      YPLT = CLWT*(CLMAX-0.5*CLDEL) - 0.6*CH
      IF(NCLEN.GT.0) THEN
      CALL PLCHAR(0.9*CH,YPLT       ,1.1*CH,CC ,0.0,1)
      ENDIF
      CALL PLCHAR(2.0*CH,YPLT       ,1.4*CH,'C',0.0,1)
      CALL PLCHAR(3.2*CH,YPLT-0.4*CH,0.9*CH,'L',0.0,1)
C
      IF(ABS(CLEXP-1.0) .GT. 0.001)
     & CALL PLNUMB(2.0*CH+1.05*CH,YPLT+1.3*CH,0.70*CH,CLEXP,0.0,1)
C
C---- a-axis for CL-a plot
      CALL NEWPEN(2)
      IF(CLMIN*CLMAX.LE.0.0) THEN
       CALL XAXIS(ALWT*ALMIN,0.0,-XAL,ALWT*ALDEL,ALMIN,ALDEL,CH2,-1)
      ELSE
       CALL XAXIS(ALWT*ALMIN,CLWT*CLMIN,-XAL,ALWT*ALDEL,ALMIN,
     &            ALDEL,CH2,-1)
      ENDIF
C
      CALL NEWPEN(3)
      XPLT = ALWT*(ALMAX - 1.5*ALDEL) - 0.5*CH
      YPLT = -4.5*CH
      CALL PLMATH(XPLT,YPLT,1.4*CH,'a',0.0,1)
C
C---- plot CL-a plot
      DO IP=1, NPOL
        CALL NEWCOLOR(ICOL(IP))
        CALL NEWPEN(IPEN)
        CALL XYLINE(NA(IP),CPOL(1,IAL,IP),CPOL(1,ICL,IP),
     &              0.0,ALWT,0.0,CLWT,ILIN(IP))
      END DO
C
C---- plot reference data
      DO ID=1, NDAT
        IF(NF(2,ID).NE.0) THEN
         CALL NEWCOLOR(IFCOL(ID))
         CALL NEWPEN(IFPEN)
         CALL XYSYMB(NF(2,ID),XYREF(1,1,2,ID),XYREF(1,2,2,ID),
     &               0.0,ALWT,0.0,CLWT,SH,IFSYM(ID))
        ENDIF
      END DO
      CALL NEWCOLOR(ICOL0)
C
      DXG = ALWT*ALDEL
      DYG = CLWT*CLDEL
      NXG = INT( XAL/(ALWT*ALDEL)    + 0.01 )
      NYG = INT( (CLMAX-CLMIN)/CLDEL + 0.01 )
      X0 = ALWT*ALMIN
C----- fine grid
      IF(LGRID) THEN
       CALL NEWPEN(1)
       X0 = ALWT*ALMIN
       Y0 = CLWT*CLMIN
       DYGF = DYG / 5.0
       NYGF = 5*NYG
       CALL PLGRID(X0,Y0, NXG,DXG, NYGF,DYGF, LMASK2 )
      ENDIF
C
C
C--- CM-alfa plot
C==================================================================
C---- CM axis for CM-a plot, skip CM plot if CMDEL=0.0
      IF(CMDEL.EQ.0) GO TO 200
C
C---- CM axis along positive CL axis (sign of CM set by max(CMMAX,CMMIN))
      IF (CMMAX.GT.0.0 .AND. CMMAX.GT.ABS(CMMIN)) THEN
       CM0 = 0.0
       CM1 = CMMAX
       DIR = 1.0
      ELSE
       CM0 = 0.0
       CM1 = CMMIN
       DIR = -1.0
      ENDIF
C
      YCM = ABS(CMWT*CM1)
      NDIG = NDIGITS(CMDEL)
C---- Offset CM axis to start at CL=0.0 or at CLmin if CLmin>0
      IF(CLMAX*CLMIN.LE.0.0) THEN
       CMOFF = 0.0 
      ELSE
       CMOFF = CLWT*CLMIN
      ENDIF
C
      CALL NEWPEN(2)
      CALL YAXIS(0.0,CMOFF,-YCM,CMWT*CMDEL,-CM0,DIR*CMDEL,CH2,NDIG)
C
      CALL NEWPEN(3)
      XPLT = -4.5*CH
      YPLT = CMOFF + CMWT*DIR*CM1 - CMWT*0.5*CMDEL - 0.6*CH
      IF(NCLEN.GT.0) THEN
      CALL PLCHAR(XPLT-0.8*CH,YPLT       ,1.1*CH,CC ,0.0,1)
      CALL PLMATH(XPLT+0.2*CH,YPLT       ,1.1*CH,'2',0.0,1)
      ENDIF
      CALL PLCHAR(XPLT+1.2*CH,YPLT       ,1.4*CH,'C',0.0,1)
      CALL PLCHAR(XPLT+2.4*CH,YPLT-0.4*CH,0.9*CH,'M',0.0,1)
C---- Offset for CM plotting
      YOFF = -CMOFF/(DIR*CMWT)
C
C---- plot CM-a plot
      DO IP=1, NPOL
        CALL NEWCOLOR(ICOL(IP))
        CALL NEWPEN(IPEN)
        CALL XYLINE(NA(IP),CPOL(1,IAL,IP),CPOL(1,ICM,IP),
     &              0.0,ALWT,YOFF,DIR*CMWT,ILIN(IP))
      END DO
C
C---- plot reference data
      DO ID=1, NDAT
        IF(NF(3,ID).NE.0) THEN
         CALL NEWCOLOR(IFCOL(ID))
         CALL NEWPEN(IFPEN)
         CALL XYSYMB(NF(3,ID),XYREF(1,1,3,ID),XYREF(1,2,3,ID),
     &               0.0,ALWT,YOFF,DIR*CMWT,SH,IFSYM(ID))
        ENDIF
      END DO
      CALL NEWCOLOR(ICOL0)
C
C
C--- transition location plot
C==================================================================
C---- re-origin for xtr plot
 200  CALL PLOT( ALWT*ALMAX + 0.05, 0.0, -3 )
      IF(XOC .EQ. 0.0) GO TO 300
C
      CALL NEWPEN(2)
      NDIG = 1
      CALL XAXIS(0.0,CLWT*CLMIN,XOC,0.5*XOC,0.0,0.5,CH2,NDIG)
C
      CALL NEWPEN(3)
      XPLT = 0.75*XOC    - 2.2*CH2
      YPLT = CLWT*CLMIN  - 4.7*CH2
      CALL PLCHAR(XPLT,YPLT,1.3*CH2,'x  /c',0.0,5)
      CALL PLCHAR(XPLT+1.2*CH2,YPLT-0.4*CH2,0.9*CH2,'tr',0.0,2)
C
C---- plot xtr/c
      DO IP=1, NPOL
        CALL NEWCOLOR(ICOL(IP))
        CALL NEWPEN(IPEN)
        DO IS=1, 2*NBL(IP)
          CALL XYLINE(NA(IP),CPOLSD(1,IS,JTN,IP),CPOL(1,ICL,IP),
     &                0.0,XOC,0.0,CLWT,ILIN(IP))
        END DO
      END DO
C
C---- plot reference data
      DO ID=1, NDAT
        IF(NF(4,ID).NE.0) THEN
         CALL NEWCOLOR(IFCOL(ID))
         CALL NEWPEN(IFPEN)
         CALL XYSYMB(NF(4,ID),XYREF(1,1,4,ID),XYREF(1,2,4,ID),
     &               0.0,XOC,0.0,CLWT,SH,IFSYM(ID))
        ENDIF
      END DO
      CALL NEWCOLOR(ICOL0)
C
C----- coarse grid lines
       CALL NEWPEN(1)
       CALL PLOT(0.0    ,CLWT*CLMIN,3)
       CALL PLOT(0.0    ,CLWT*CLMAX,2)
       CALL PLOT(0.5*XOC,CLWT*CLMIN,3)
       CALL PLOT(0.5*XOC,CLWT*CLMAX,2)
       CALL PLOT(    XOC,CLWT*CLMIN,3)
       CALL PLOT(    XOC,CLWT*CLMAX,2)
C
       DYG = CLWT*CLDEL
       Y0  = CLWT*CLMIN
       NYG = INT( (CLMAX-CLMIN)/CLDEL + 0.01 )
       DO K=0, NYG
         YL = Y0 + DYG*FLOAT(K)
         CALL PLOT(0.0,YL,3)
         CALL PLOT(XOC,YL,2)
       END DO
C
C----- fine grid 
      IF(LGRID) THEN
       CALL NEWPEN(1)
       DXG =  XOC*0.5   / 5.0
       DYG = CLWT*CLDEL / 5.0
       X0 = 0.0
       Y0 = CLWT*CLMIN
       NXG = 10
       NYG = INT( (CLMAX-CLMIN)/CLDEL + 0.01 ) * 5
       CALL PLGRID(X0,Y0, NXG,DXG, NYG,DYG, LMASK2 )
C
      ENDIF
C
C
C==================================================================
C---- aerodynamic center
      IF(LAECEN) THEN
C
      CALL NEWPEN(2)
      XPLT = 0.25*XOC   - 2.2*CH2
      YPLT = CLWT*CLMIN - 4.7*CH2
      CALL PLCHAR(XPLT,YPLT,1.3*CH2,'x  /c',0.0,5)
      CALL PLCHAR(XPLT+1.2*CH2,YPLT-0.4*CH2,0.9*CH2,'ac',0.0,2)
C
      CHS = 0.25*CH2
C
C---- plot xac/c
      DO IP=1, NPOL
        CALL NEWCOLOR(ICOL(IP))
        CALL NEWPEN(2)
        DO IA = 1, NA(IP)-1
          DCM =  CPOL(IA+1,ICM,IP) - CPOL(IA,ICM,IP)
          DCL =  CPOL(IA+1,ICL,IP) - CPOL(IA,ICL,IP)
          CLA = (CPOL(IA+1,ICL,IP) + CPOL(IA,ICL,IP))*0.5
C
          IF(DCL .NE. 0.0) THEN
           XAC = 0.25 - DCM/DCL
          ELSE
           XAC = 0.0
          ENDIF
C
          IF(XAC .GT. 0.0 .AND.
     &       XAC .LT. 1.0      ) THEN
           CALL PLSYMB(XAC*XOC,CLA*CLWT,CHS,5,0.0,0)
          ENDIF
        END DO
      END DO
C
      ENDIF
C
      CALL NEWCOLOR(ICOL0)
C
C==================================================================
C---- code and version identifier
 300  CONTINUE
      CHI = 0.75*CH2
      CALL NEWPEN(2)
      XPLT = XOC        - 12.0*CHI
      YPLT = CLWT*CLMAX +  0.5*CHI
      CALL PLCHAR(XPLT        ,YPLT,CHI,CODE   ,0.0,5)
      CALL PLCHAR(XPLT+6.0*CHI,YPLT,CHI,'V'    ,0.0,1)
      CALL PLNUMB(XPLT+8.0*CHI,YPLT,CHI,VERSION,0.0,2)
C
      CALL PLFLUSH
C---- reset scale factors
      CALL NEWFACTORS(XSZ,YSZ)
C
      RETURN
      END ! POLPLT
 


      SUBROUTINE POLLAB(NPOL, NAME ,ICOL,
     &                 IMATYP, IRETYP,
     &                 MACH, REYN, ACRIT,
     &                 TITLE,
     &                 XPLT0,YPLT0, PLOTAR, CH,CH2, 
     &                 LLIST, CCLEN,NCLEN )
C
      INCLUDE 'PINDEX.INC'
C
      CHARACTER*(*) NAME(NPOL)
      CHARACTER*(*) TITLE, CCLEN
C
      DIMENSION ICOL(NPOL), IMATYP(NPOL),IRETYP(NPOL)
      REAL MACH
      DIMENSION MACH(NPOL), REYN(NPOL), ACRIT(NPOL)
      LOGICAL LLIST
C----------------------------------------------
C     Generates label for polar plot
C----------------------------------------------
      CH3 = 0.90*CH2
      CH4 = 1.10*CH2
C
C---- y-spacing for label lines
      YSPC = 1.9*CH4
C
C...Put up title
C
      XPLT = XPLT0 - CH2
      YPLT = YPLT0 + 0.6*CH4
      IF(LLIST) THEN
        YPLT = YPLT + YSPC*(NPOL+1)
       ELSE
        YPLT = YPLT + 0.5*CH4
      ENDIF
      CALL NEWPEN(3)
      LENT = LEN(TITLE)
      CALL PLCHAR(XPLT,YPLT,1.2*CH4,TITLE,0.0,LENT)
C
      IF(.NOT.LLIST) RETURN
C
C
C...Put up polar identification data: name, flow conditions
      NMAX = 0
      DO IP = 1, NPOL
        CALL STRIP(NAME(IP),NNAME)
        NMAX = MAX(NMAX,NNAME)
      END DO
C
      DO IP = 1, NPOL
C
      CALL NEWCOLOR(ICOL(IP))
C
      XPLT = XPLT0
      YPLT = YPLT0 + YSPC*(NPOL-IP+1)
C
      CALL NEWPEN(3)
      CALL PLCHAR(XPLT,YPLT,CH4,NAME(IP),0.0,NMAX)
      XPLT = XPLT + CH4*FLOAT(NMAX)
C
      CALL NEWPEN(2)
C
       ITYP = IRETYP(IP)
       IF(ITYP.EQ.1) THEN
        CALL PLCHAR(XPLT,YPLT,CH3,'   Re = '   ,0.0,  8)
        XPLT = XPLT + CH3*8.0
       ELSE IF(ITYP.EQ.2) THEN
        CALL PLCHAR(XPLT,YPLT,CH3,'   Re CL = ',0.0, 11)
        CALL PLMATH(XPLT,YPLT,CH3,'     R   = ',0.0, 11)
        XPLT = XPLT + CH3*11.0
       ELSE IF(ITYP.EQ.3) THEN
        CALL PLCHAR(XPLT,YPLT,CH3,'   Re CL = ',0.0, 11)
        XPLT = XPLT + CH3*11.0
       ENDIF
       CALL PLNUMB(XPLT,YPLT,CH3,REYN(IP),0.0,-1)
       IF(NCLEN.GT.0) THEN
        CALL PLCHAR(999.,YPLT,CH3,'/'  ,0.0,1)
        CALL PLCHAR(999.,YPLT,CH3,CCLEN,0.0,NCLEN)
        XPLT = XPLT + CH3*FLOAT(1+NCLEN)
       ENDIF
       XPLT = XPLT + CH3*7.0
C
       ITYP = IMATYP(IP)
       IF(ITYP.EQ.1) THEN
        CALL PLCHAR(XPLT,YPLT,CH3,'   Ma = '   ,0.0,  8)
        XPLT = XPLT + CH3*8.0
       ELSE IF(ITYP.EQ.2) THEN
        CALL PLCHAR(XPLT,YPLT,CH3,'   Ma CL = ',0.0, 11)
        CALL PLMATH(XPLT,YPLT,CH3,'     R   = ',0.0, 11)
        XPLT = XPLT + CH3*11.0
       ELSE IF(ITYP.EQ.3) THEN
        CALL PLCHAR(XPLT,YPLT,CH3,'   Ma CL = ',0.0, 11)
        XPLT = XPLT + CH3*11.0
       ENDIF
       CALL PLNUMB(XPLT,YPLT,CH3,    MACH(IP)  ,0.0,3)
       XPLT = XPLT + CH3*5.0
C
       CALL PLCHAR(XPLT,YPLT,    CH3,'   N',0.0,4)
       XPLT = XPLT + CH3*4.0
       CALL PLCHAR(XPLT,YPLT,0.8*CH3,'crit',0.0,4)
       XPLT = XPLT + CH3*3.2
       CALL PLCHAR(XPLT,YPLT,    CH3,' = ' ,0.0,3)
       XPLT = XPLT + CH3*3.0
       CALL PLNUMB(XPLT,YPLT,    CH3,ACRIT(IP) ,0.0,3)
       XPLT = XPLT + CH3*6.0
C
      END DO
C
      RETURN
      END ! POLLAB



      SUBROUTINE GETVAR(NPOL,NAME,REYN,MACH,ACRIT,
     &                  NAMVAR,REYVAR,MACVAR,ACRVAR)
      CHARACTER*(*) NAME
      REAL MACH
      LOGICAL NAMVAR,REYVAR,MACVAR,ACRVAR
C
      DIMENSION NAME(NPOL),REYN(NPOL),MACH(NPOL),ACRIT(NPOL)
C
      NAMVAR = .FALSE.
      MACVAR = .FALSE.
      REYVAR = .FALSE.
      ACRVAR = .FALSE.
C
      DO IP=1, NPOL-1
        IF(NAME(IP) .NE. NAME(IP+1)) THEN
         NAMVAR = .TRUE.
         RETURN
        ENDIF
      END DO
C
      DO IP=1, NPOL-1
        IF(MACH(IP) .NE. MACH(IP+1)) THEN
         MACVAR = .TRUE.
         RETURN
        ENDIF
      END DO
C
      DO IP=1, NPOL-1
        IF(REYN(IP) .NE. REYN(IP+1)) THEN
         REYVAR = .TRUE.
         RETURN
        ENDIF
      END DO
C
      DO IP=1, NPOL-1
        IF(ACRIT(IP) .NE. ACRIT(IP+1)) THEN
         ACRVAR = .TRUE.
         RETURN
        ENDIF
      END DO
C
ccc   NAMVAR = .TRUE.
      RETURN
      END ! GETVAR


      INTEGER FUNCTION NDIGITS(X)
C...Returns number of significant (non-zero) fractional digits
      NDIGITS = 0
      XMAG = ABS(X)
      IF(XMAG.EQ.0.) RETURN
   1  XDIF = XMAG-IFIX(XMAG)
      IF(XDIF.LT.1.E-5 .OR. 1.0-XDIF.LT.1.E-5) RETURN
        NDIGITS = NDIGITS+1
        XMAG = 10.*XMAG 
        GO TO 1
      END


      SUBROUTINE VEPPLT(NAX,NPOL,NA,VPOL,
     &                  REYN,MACH,ACRIT, NAME ,ICOL,ILIN,
     &                  IMATYP,IRETYP,
     &                  TITLE,CODE,VERSION,
     &                  PLOTAR, CH,CH2,
     &                  LGRID,LLIST,LEGND,
     &                  VPOLPLF )
C----------------------------------------------------------------
C     Generates velocity-polar plot
C----------------------------------------------------------------
      CHARACTER*(*) NAME(NPOL)
      CHARACTER*(*) CODE, TITLE
      LOGICAL LGRID, LLIST, LEGND
C
      INTEGER NA(NPOL), 
     &        ICOL(NPOL), ILIN(NPOL),
     &        IMATYP(NPOL),IRETYP(NPOL)
      REAL VPOL(NAX,2,NPOL)
      REAL VPOLPLF(3,*)
      REAL REYN(NPOL), MACH(NPOL), ACRIT(NPOL)
C----------------------------------------------------------------
      LOGICAL NAMVAR,REYVAR,MACVAR,ACRVAR
      REAL XLIN(3), YLIN(3)
      CHARACTER*1 CC
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      CALL GETVAR(NPOL,NAME,REYN,MACH,ACRIT,
     &            NAMVAR,REYVAR,MACVAR,ACRVAR)
C
C---- polar and data-symbol pen width
      IPEN = 4
      IFPEN = 3
C
C---- symbol height for data
      SH = 0.7*CH2
C
C---- unpack plot limit array
      VHMIN = VPOLPLF(1,1)
      VHMAX = VPOLPLF(2,1)
      VHDEL = VPOLPLF(3,1)
C                 
      VZMIN = VPOLPLF(1,2)
      VZMAX = VPOLPLF(2,2)
      VZDEL = VPOLPLF(3,2)
C                 

c      WRITE(*,*) VPOLPLF(1,1),VPOLPLF(2,1),VPOLPLF(3,1)
c      WRITE(*,*) VPOLPLF(1,2),VPOLPLF(2,2),VPOLPLF(3,2)


C---- Get scale factor and set scale factor to 0.9 of current to fit plots
      CALL GETFACTORS(XSZ,YSZ)
      CALL NEWFACTORS(0.9*XSZ,0.9*YSZ)
C
C---- Set sane scale factors for axes
      VHWT = 1.0
      VZWT = 1.0
C
      VHRANGE = VHMAX-VHMIN
      IF(VHRANGE.NE.0.0) THEN
        VHWT = 1.0 / VHRANGE
      ENDIF
C
      VZRANGE = VZMAX-VZMIN
      IF(VZRANGE.NE.0.0) THEN
        VZWT = PLOTAR / VZRANGE
      ENDIF
C
C
C---- number of text lines to be plotted in upper right legend in VH-VZ plot
      LINBOX = NDAT
      IF(LEGND.AND. NPOL.GT.1) LINBOX = LINBOX + NPOL + 1
      DYBOX = CH2*(2.0*FLOAT(LINBOX) + 1.0)
C
C---- allow # CH2 character string width in label box
      NCHBOX = 18
      DXBOX = FLOAT(NCHBOX)*CH2
C

C---- set default color index
      CALL GETCOLOR(ICOL0)
C---- reorigin for VZMIN,VHMIN
      CALL PLOT(-VHWT*VHMIN,-VZWT*VZMIN,-3)
C
C---- put Polar labels above plots 
C     Labels contain: Title
C           airfoils: Name, Mach, Re, and Ncrit
C
      XPLT0 = VHWT*VHMIN
      YPLT0 = VZWT*VZMAX
      CALL POLLAB(NPOL, NAME ,ICOL,
     &           IMATYP,IRETYP,
     &           MACH, REYN, ACRIT,
     &           TITLE,
     &           XPLT0,YPLT0, PLOTAR, CH,CH2, 
     &           LLIST, ' ',0 )
C
      CALL NEWCOLOR(ICOL0)
C
C
C--- VH-VZ plot
C==================================================================
C---- VZ axis for VH-VZ polar
      CALL NEWPEN(2)
      NDIG = NDIGITS(VZDEL)
      CALL YAXIS(VHWT*VHMIN,VZWT*VZMIN,PLOTAR,VZWT*VZDEL,
     &           VZMIN,VZDEL,CH2,NDIG)
C
      CALL NEWPEN(3)
      XPLT = VHWT* VHMIN            - 3.2*CH
      YPLT = VZWT*(VZMAX-0.5*VZDEL) - 0.6*CH
      CALL PLCHAR(XPLT       ,YPLT       ,1.4*CH,'V',0.0,1)
      CALL PLCHAR(XPLT+1.2*CH,YPLT-0.4*CH,0.9*CH,'z',0.0,1)
C
C---- VH axis for VH-VZ polar
      CALL NEWPEN(2)
      NDIG = NDIGITS(VHDEL)
      CALL XAXIS(VHWT*VHMIN,VZWT*VZMIN,1.0,VHWT*VHDEL,
     &           VHMIN,VHDEL,CH2,NDIG)
C
      CALL NEWPEN(3)
      NXL = INT((VHMAX-VHMIN)/VHDEL + 0.5)
      XPLT = VHWT*(VHMAX - (FLOAT((NXL+1)/2) - 0.5)*VHDEL) - 0.5*CH2
      YPLT = VZWT* VZMIN - 4.8*CH2
      CALL PLCHAR(XPLT,YPLT,1.4*CH,'V',0.0,1)
C
C---- set up for coarse grid lines
      CALL NEWPEN(1)
      DXG = VHWT*VHDEL
      DYG = VZWT*VZDEL
C
C---- check for legend box at top left of VH-VZ grid area
      NXGBOX = INT( DXBOX/(DXG/5.0) ) + 1
      NYGBOX = INT( DYBOX/(DYG/5.0) ) + 1
      IF (LINBOX.EQ.0) THEN
        NXGBOX = 0
        NYGBOX = 0
      ENDIF
      DXGBOX = (DXG/5.0) * FLOAT(NXGBOX)
      DYGBOX = (DYG/5.0) * FLOAT(NYGBOX)
C
      X0 = VHWT*VHMIN
      Y0 = VZWT*VZMIN
      NXG = INT( 1.0/(VHWT*VHDEL)    + 0.01 )
      NYG = INT( (VZMAX-VZMIN)/VZDEL + 0.01 )
C
C---- Put legend data in legend box in upper right of VH/VZ plot
      IF(LEGND) THEN
C
      XBASE = VHWT*VHMAX - DXGBOX
      YLINE = VZWT*VZMAX - 2.0*CH2
      CALL NEWPEN(3)
C
      IF(NAMVAR) THEN
       XPLT = XBASE + 6.0*CH2
       YPLT = YLINE
       CALL PLCHAR(XPLT    ,YPLT,    CH2,'Airfoil',0.0,7)
       YLINE = YLINE - 2.25*CH2
      ENDIF
C
      IF(REYVAR) THEN
       XPLT = XBASE + 7.5*CH2
       YPLT = YLINE
       ITYP = IRETYP(1)
       IF(ITYP.EQ.1) THEN
        CALL PLCHAR(XPLT        ,YPLT,    CH2,'Re'  ,0.0,2)
       ELSE IF(ITYP.EQ.2) THEN
        CALL PLMATH(XPLT-1.0*CH2,YPLT,    CH2,'  R  ',0.0,5)
        CALL PLCHAR(XPLT-1.0*CH2,YPLT,    CH2,'Re C' ,0.0,4)
        CALL PLCHAR(999.        ,999.,0.7*CH2,    'L',0.0,1)
       ELSE IF(ITYP.EQ.3) THEN
        CALL PLMATH(XPLT-1.0*CH2,YPLT,    CH2,'  #  ',0.0,5)
        CALL PLCHAR(XPLT-1.0*CH2,YPLT,    CH2,'Re C' ,0.0,4)
        CALL PLCHAR(999.        ,999.,0.7*CH2,    'L',0.0,1)
       ENDIF
       YLINE = YLINE - 2.25*CH2
      ENDIF
C
      IF(ACRVAR) THEN
       XPLT = XBASE + 8.0*CH2
       YPLT = YLINE
       CALL PLCHAR(XPLT,YPLT,    CH2,'N'   ,0.0,1)
       CALL PLCHAR(999.,999.,0.7*CH2,'crit',0.0,4)
       YLINE = YLINE - 2.25*CH2
      ENDIF
C
      IF(MACVAR) THEN
       XPLT = XBASE + 7.5*CH2
       YPLT = YLINE
       ITYP = IMATYP(1)
       IF(ITYP.EQ.1) THEN
        CALL PLCHAR(XPLT        ,YPLT,    CH2,'Ma'  ,0.0,2)
       ELSE IF(ITYP.EQ.2) THEN
        CALL PLMATH(XPLT-1.0*CH2,YPLT,    CH2,'  R  ',0.0,5)
        CALL PLCHAR(XPLT-1.0*CH2,YPLT,    CH2,'Ma C' ,0.0,4)
        CALL PLCHAR(999.        ,999.,0.7*CH2,    'L',0.0,1)
       ELSE IF(ITYP.EQ.3) THEN
        CALL PLMATH(XPLT-1.0*CH2,YPLT,    CH2,'  #  ',0.0,5)
        CALL PLCHAR(XPLT-1.0*CH2,YPLT,    CH2,'Ma C' ,0.0,4)
        CALL PLCHAR(999.        ,999.,0.7*CH2,    'L',0.0,1)
       ENDIF
       YLINE = YLINE - 2.25*CH2
      ENDIF
C
      ENDIF
C
C---- plot VH-VZ polar(s)
      DO IP=1, NPOL
        CALL NEWCOLOR(ICOL(IP))
        CALL NEWPEN(IPEN)
        CALL XYLINE(NA(IP),VPOL(1,1,IP),VPOL(1,2,IP),
     &              0.,VHWT,0.,VZWT,ILIN(IP))
      END DO
C
C---- label each polar with legend
      IF(LEGND .AND. (NAMVAR .OR. REYVAR .OR. ACRVAR .OR. MACVAR)) THEN
       DO IP=1, NPOL
         CALL NEWCOLOR(ICOL(IP))
         XLIN(1) = XBASE +     CH2
         XLIN(2) = XBASE + 3.0*CH2
         XLIN(3) = XBASE + 6.0*CH2
         YLIN(1) = YLINE + 0.5*CH2
         YLIN(2) = YLINE + 0.5*CH2
         YLIN(3) = YLINE + 0.5*CH2
         CALL NEWPEN(IPEN)
         CALL XYLINE(3,XLIN,YLIN,0.0,1.0,0.0,1.0,ILIN(IP))
         CALL NEWPEN(2)
         XPT = XBASE + 7.5*CH2
         IF(NAMVAR) CALL PLCHAR(XPT,YLINE,.8*CH2,NAME(IP) ,0.,14)
         IF(REYVAR) CALL PLNUMB(XPT,YLINE,.8*CH2,REYN(IP) ,0.,-1)
         IF(ACRVAR) CALL PLNUMB(XPT,YLINE,.8*CH2,ACRIT(IP),0., 3)
         IF(MACVAR) CALL PLNUMB(XPT,YLINE,.8*CH2,MACH(IP) ,0., 3)
         YLINE = YLINE - 2.0*CH2
       END DO
       YLINE = YLINE - 0.5*CH2
C
      ENDIF
C
       CALL NEWCOLOR(ICOL0)
       CALL NEWPEN(1)
C
C----- plot vertical coarse grid lines around label box
       DO K = 0, NXG
        DXL = VHWT*VHDEL*FLOAT(K)
        XL = X0 + DXL
        CALL PLOT(XL,Y0,3)
        IF(XL .LT. VHWT*VHMAX-0.999*DXGBOX) THEN
         CALL PLOT(XL, Y0 + DYG*FLOAT(NYG)       , 2)
        ELSE
         CALL PLOT(XL, Y0 + DYG*FLOAT(NYG)-DYGBOX, 2)
        ENDIF
       END DO
C
C----- plot horizontal coarse grid lines around label box
       DO K = 0, NYG
         DYL = VZWT*VZDEL*FLOAT(K)
         YL = Y0 + DYL
         CALL PLOT(X0,YL,3)
         IF(YL .LT. VZWT*VZMAX-0.999*DYGBOX) THEN
          CALL PLOT(X0 + DXG*FLOAT(NXG), YL, 2)
         ELSE
          CALL PLOT(X0 + DXG*FLOAT(NXG)-DXGBOX, YL, 2)
         ENDIF
       END DO
C
C---- plot edges of label box
       X0 = VHWT*VHMAX
       Y0 = VZWT*VZMAX
       CALL PLOT(X0       , Y0       , 3)
       CALL PLOT(X0-DXGBOX, Y0       , 2)
       CALL PLOT(X0-DXGBOX, Y0-DYGBOX, 2)
       CALL PLOT(X0       , Y0-DYGBOX, 2)
       CALL PLOT(X0       , Y0       , 2)
C
C----- fine grid
      IF(LGRID) THEN
       CALL NEWPEN(1)
       DXG = VHWT*VHDEL / 5.0
       DYG = VZWT*VZDEL / 5.0
       X0 = VHWT*VHMIN
       Y0 = VZWT*VZMIN
C
C----- plot fine grid left of the label box
       NXGF = 5*NXG - NXGBOX
       NYGF = 5*NYG
       CALL PLGRID(X0,Y0, NXGF,DXG, NYGF,DYG, LMASK2 )
C
C=---- plot fine grid under the label box, if present
       X0 = VHWT*VHMAX - DXGBOX
       NXGF =         NXGBOX
       NYGF = 5*NYG - NYGBOX
       IF(NXGF.GT.0) CALL PLGRID(X0,Y0, NXGF,DXG, NYGF,DYG, LMASK2 )
      ENDIF
C
      CALL NEWCOLOR(ICOL0)
C
C==================================================================
C---- code and version identifier
 300  CONTINUE
      CHI = 0.75*CH2
      CALL NEWPEN(2)
      XPLT = 1.0        - 12.0*CHI
      YPLT = VZWT*VZMAX +  0.5*CHI
      CALL PLCHAR(XPLT        ,YPLT,CHI,CODE   ,0.0,5)
      CALL PLCHAR(XPLT+6.0*CHI,YPLT,CHI,'V'    ,0.0,1)
      CALL PLNUMB(XPLT+8.0*CHI,YPLT,CHI,VERSION,0.0,2)
C
      CALL PLFLUSH
C---- reset scale factors
      CALL NEWFACTORS(XSZ,YSZ)
C
      RETURN
      END ! VEPPLT
