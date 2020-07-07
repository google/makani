C***********************************************************************
C    Module:  plsubs.f
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

      SUBROUTINE PLROOT(XORG,YORG, 
     &                  IRUN1, IRUN2, IRCOLOR,
     &                  JEMAX,NEIGEN, EVAL, LPROOT, LPRNUM,
     &                  UNCHT, PAR,CS, LPGRID,
     &                  XMIN,XMAX,XDEL,XFAC,YMIN,YMAX,YDEL,YFAC)
      INTEGER IRCOLOR(*)
      INTEGER NEIGEN(*)
      COMPLEX EVAL(JEMAX,*)
      CHARACTER*(*) UNCHT
      LOGICAL LPROOT(JEMAX,*), LPRNUM(JEMAX,*), LPGRID
C-----------------------------------------------------------------
C     Makes root-locus plot
C     If XMIN=XMAX, then XMIN,XMAX,XDEL are set here.
C     If YMIN=YMAX, then YMIN,YMAX,YDEL are set here.
C-----------------------------------------------------------------
      DATA TPI / 6.2831853071795864769 /
      INCLUDE 'MASKS.INC'
C
C---- label character size
      CSL = 1.5*CS
      CSU = 1.2*CS
      CSA = 1.0*CS
      CSS = 0.5*CS
      CSN = 0.8*CS
C
      NRUN = IRUN2 - IRUN1 + 1

C---- root symbol index
      ISYMB = 1
C
      CALL GETCOLOR(ICOL0)
C
      IF(XMIN.EQ.XMAX) THEN
       XMIN =  1.0E23
       XMAX = -1.0E23
       DO IR = IRUN1, IRUN2
         DO KEIG = 1, NEIGEN(IR)
           IF(LPROOT(KEIG,IR)) THEN
            XMIN = MIN(XMIN,REAL(EVAL(KEIG,IR)))
            XMAX = MAX(XMAX,REAL(EVAL(KEIG,IR)))
           ENDIF
         ENDDO
       ENDDO
       CALL AXISADJ(XMIN,XMAX, XTOT, XDEL, NANN)
      ENDIF
C
      IF(YMIN.EQ.YMAX) THEN
       YMIN =  1.0E23
       YMAX = -1.0E23
       DO IR = IRUN1, IRUN2
         DO KEIG = 1, NEIGEN(IR)
           IF(LPROOT(KEIG,IR)) THEN
            YMIN = MIN(YMIN,IMAG(EVAL(KEIG,IR)))
            YMAX = MAX(YMAX,IMAG(EVAL(KEIG,IR)))
           ENDIF
         ENDDO
       ENDDO
       YMIN = 0.0     !##
       CALL AXISADJ(YMIN,YMAX, YTOT, YDEL, NANN)
      ENDIF
C
C---- make sure aspect ratio of plot isn't absurd
      IF(XMAX-XMIN .LT. 2.0*YDEL) THEN
       XMIN = XMAX - 2.0*YDEL
       CALL AXISADJ(XMIN,XMAX, XTOT, XDEL, NANN)
      ENDIF
      IF(YMAX-YMIN .LT. 2.0*XDEL) THEN
cc       YMIN = 0.5*(YMIN+YMAX) - XDEL  !##
       YMAX = YMIN + 2.0*XDEL
       CALL AXISADJ(YMIN,YMAX, YTOT, YDEL, NANN)
      ENDIF
C
C---- annotations for Hz axis
      YTMIN = YMIN/TPI
      YTMAX = YMAX/TPI
      CALL AXISADJ2(YTMIN,YTMAX, YTTOT, YTDEL, NANN)
      IF(YTDEL.GT.0.49*(YTMAX-YTMIN)) THEN
       YTDEL = 0.5*YTDEL
       NANN = 2*NANN
      ENDIF
      IF(YTMIN.LT.YMIN/TPI) YTMIN = YTMIN + YTDEL
      IF(YTMAX.GT.YMAX/TPI) YTMAX = YTMAX - YTDEL
C
      XFAC = 1.0/(XMAX-XMIN)
      YFAC = PAR/(YMAX-YMIN)
C
      SFAC = MIN( XFAC , YFAC )
      XFAC = SFAC
      YFAC = SFAC
C
      XLEN = (XMAX-XMIN)*XFAC
      YLEN = (YMAX-YMIN)*YFAC
C
      YTFAC = YFAC*TPI
C
      CALL NEWPEN(1)
      IF(LPGRID) THEN
       DXG = 0.5*XDEL*XFAC
       DYG = 0.5*YDEL*YFAC
C
       NXG = INT( 2.0*(XMAX-XMIN)/XDEL + 0.5 )
       NYG = INT( 2.0*(YMAX-YMIN)/YDEL + 0.5 )
       CALL PLGRID(XORG,YORG, NXG,DXG, NYG,DYG, LMASK2 )
C
      ELSE
       CALL PLOT(XORG     ,YORG+YLEN,3)
       CALL PLOT(XORG+XLEN,YORG+YLEN,2)
       CALL PLOT(XORG+XLEN,YORG     ,3)
       CALL PLOT(XORG+XLEN,YORG+YLEN,2)
C
      ENDIF
C
      CALL NEWPEN(2)
      CALL XAXIS(XORG     ,YORG,XLEN, XDEL*XFAC,  XMIN, XDEL, CSA,-2)
      CALL YAXIS(XORG     ,YORG,YLEN, YDEL*YFAC,  YMIN, YDEL, CSA,-2)
      CALL YAXIS(XORG+XLEN,YORG-YMIN*YFAC+YTMIN*YTFAC,
     &          (YTMAX-YTMIN)*YTFAC,YTDEL*YTFAC, YTMIN,YTDEL,-CSA,-2)
C
C---- plot x,y axes as heavy dotted lines
      CALL NEWPEN(2)
      CALL NEWPAT(LMASK1)
      CALL PLOT(XORG     ,YORG-YMIN*YFAC,3)
      CALL PLOT(XORG+XLEN,YORG-YMIN*YFAC,2)
      CALL PLOT(XORG-XMIN*XFAC,YORG     ,3)
      CALL PLOT(XORG-XMIN*XFAC,YORG+YLEN,2)
      CALL NEWPAT(LMASK0)
C
C---- plot axis parameter name annotations
      CALL NEWPEN(3)
      XL = XORG + XLEN - 0.5*CSL - 1.5*XDEL*XFAC
      YL = YORG        - 2.3*CSL
      CALL PLMATH(XL  ,YL, CSL,'s'    ,0.0, 1)
      XL = XORG + XLEN - 1.5*CSU - 0.5*XDEL*XFAC
      CALL PLCHAR(XL  ,YL, CSU,'1/'   ,0.0, 2)
      CALL PLCHAR(999.,YL, CSU, UNCHT ,0.0,-1)
C
      XL = XORG        - 3.0*CSL
      YL = YORG + YLEN - 0.5*CSL - 0.5*YDEL*YFAC
      CALL PLMATH(XL  ,YL, CSL,'w',0.0, 1)
      XL = XORG        - 4.2*CSU
      YL = YORG + YLEN - 0.5*CSL - 1.5*YDEL*YFAC
      CALL PLCHAR(XL  ,YL, CSU,'1/',0.0,2)
      CALL PLCHAR(999.,YL, CSU, UNCHT,0.0,-1)
C
      XL = XORG + XLEN      + 2.0*CSL
      YL = YORG - YMIN*YFAC - 0.5*CSL + (YTMAX - 0.5*YTDEL)*YTFAC
      CALL PLMATH(XL  ,YL, CSL,'w  p',0.0, 4)
      CALL PLCHAR(XL  ,YL, CSL,' /2 ',0.0, 4)
      XL = XORG + XLEN      + 1.2*CSU
      YL = YORG - YMIN*YFAC - 0.5*CSL + (YTMAX - 1.5*YTDEL)*YTFAC
      CALL PLCHAR(XL  ,YL, CSU,'cycles/',0.0,7)
      CALL PLCHAR(999.,YL, CSU, UNCHT,0.0,-1)
C
C---- plot root symbols
      XOFF = XMIN - XORG/XFAC
      YOFF = YMIN - YORG/YFAC
      DO IR = IRUN1, IRUN2
        DO KEIG = 1, NEIGEN(IR)
          IF(LPROOT(KEIG,IR)) THEN
           XEV = REAL(EVAL(KEIG,IR))
           YEV = IMAG(EVAL(KEIG,IR))
           IF(YEV .GE. -1.0E-4) THEN
            CALL NEWPEN(5)
            CALL NEWCOLOR(IRCOLOR(IR))
            XPLT = (XEV - XOFF)*XFAC
            YPLT = (YEV - YOFF)*YFAC
            CALL XYSYMB(1,XPLT,YPLT,0.0,1.0,0.0,1.0,CSS,ISYMB)
            IF(LPRNUM(KEIG,IR)) THEN
             CALL NEWPEN(2)
             XNUM = XPLT + 0.85*CSS
             YNUM = YPLT + 0.85*CSS
             CALL PLNUMB(XNUM,YNUM,CSN,FLOAT(IR),0.0,-1)
            ENDIF
           ENDIF
          ENDIF
        ENDDO
      ENDDO
      CALL NEWCOLOR(ICOL0)
C
      CALL PLFLUSH
C
      RETURN
      END ! PLROOT



      SUBROUTINE PLTPAR(XPLT,YPLT, IRUN1, IRUN2,
     &                  PARVAL,LPPAR,
     &                  IRCOLOR, CSIZ, DELX, DELY)
C-----------------------------------------------
C     Plots operating conditions in table form
C-----------------------------------------------
      INCLUDE 'AINDEX.INC'
C
      REAL PARVAL(IPTOT,*)
      LOGICAL LPPAR(IPTOT)
      INTEGER IRCOLOR(*)
C
C
      INTEGER NPDIG(IPTOT), NPWID(IPTOT)
      REAL XPAR(IPTOT), FACPAR(IPTOT)
C
      DATA DTR / 0.0174532925 /
C
C
      CS  =     CSIZ
      CSL = 1.3*CSIZ
C
C---- set number of significant digits for each variable
      DO IP = 1, IPTOT
        NPDIG(IP) = 4
      ENDDO
C
C---- set total number of digits for each variable (including decimal point)
      DO IP = 1, IPTOT
        NPWID(IP) = 3 + NPDIG(IP)
      ENDDO
C
C---- set scale factors of displayed value
      DO IP = 1, IPTOT
        FACPAR(IP) = 1.0
      ENDDO
cc      FACPAR(IPALFA) = 1.0/DTR
cc      FACPAR(IPBETA) = 1.0/DTR
cc      FACPAR(IPTHE ) = 1.0/DTR
cc      FACPAR(IPPHI ) = 1.0/DTR
C
C---- starting x-position
      XPOS = XPLT + 2.5*CS + DELX
C
C---- set starting x position for each variable
      DO IP = 1, IPTOT
        IF(LPPAR(IP)) THEN
         XPAR(IP) = XPOS
         XPOS = XPOS + CS*NPWID(IP) + DELX
        ELSE
         XPAR(IP) = 0.
        ENDIF
      ENDDO
C
C
      CALL GETCOLOR(ICOL0)
C
      Y = YPLT
      DO IR = IRUN2, IRUN1, -1
        Y = Y + DELY
C
        CALL NEWCOLOR(IRCOLOR(IR))
C
        CALL NEWPEN(2)
        X = XPLT
        CALL PLNUMB(X,Y,CS,FLOAT(IR),0.0,-1)
        CALL PLCHAR(999.,Y,CS,':',0.0,1)
C
C------ plot numerical parameter values
        CALL NEWPEN(2)
        DO IP = 1, IPTOT
          IF(LPPAR(IP)) THEN
           X = XPAR(IP)
           RNUM = PARVAL(IP,IR)*FACPAR(IP)
           CALL PLNUMS(X,Y,CS,RNUM,0.0,NPDIG(IP))
          ENDIF
        ENDDO
      ENDDO
C
      CALL NEWCOLOR(ICOL0)
      CALL NEWPEN(3)
C
      Y = Y + DELY + 0.3*CS
C
      DO IP = 1, IPTOT
        IF(LPPAR(IP)) THEN
         X = XPAR(IP) + 0.5*CS*FLOAT(NPWID(IP)-3) - 0.5*CSL
         CALL PPNAME(X,Y,CSL, IP)
        ENDIF
      ENDDO
C
C---- save upper Y limit for passing back
      YPLT = Y + CSL
C
      RETURN
      END ! PLTPAR



      SUBROUTINE PPNAME(X,Y,CSIZ,IP)
      INCLUDE 'AINDEX.INC'
C------------------------------------------------------------------
C     Plots symbol name of parameter specified by IP index
C
C      X,Y   position of lower-left corner of symbol
C      CSIZ  size of symbol
C      IP    index of parameter
C------------------------------------------------------------------
      CSP =      CSIZ
      CSC = 0.70*CSIZ
C
      CALL GETLASTXY(XPLT,YPLT)
      IF(X .NE. 999.0) XPLT = X
      IF(Y .NE. 999.0) YPLT = Y
C
C------ plot scalar parameter name
C
      IF    (IP.EQ.IPALFA) THEN
       CALL PLMATH(XPLT,YPLT,CSP,'a"' ,0.0,2)
C
      ELSEIF(IP.EQ.IPBETA) THEN
       CALL PLMATH(XPLT,YPLT,CSP,'b"' ,0.0,2)
C
      ELSEIF(IP.EQ.IPROTX) THEN
       CALL PLCHAR(XPLT,YPLT,CSP,'p'  ,0.0,1)
C
      ELSEIF(IP.EQ.IPROTY) THEN
       CALL PLCHAR(XPLT,YPLT,CSP,'q'  ,0.0,1)
C
      ELSEIF(IP.EQ.IPROTZ) THEN
       CALL PLCHAR(XPLT,YPLT,CSP,'r'  ,0.0,1)
C
      ELSEIF(IP.EQ.IPCL  ) THEN
       CALL PLCHAR(XPLT        ,YPLT        ,CSP,'C' ,0.0,1)
       CALL PLCHAR(XPLT+0.9*CSP,YPLT-0.4*CSP,CSC, 'L',0.0,1)
C
      ELSEIF(IP.EQ.IPCD0 ) THEN
       CALL PLCHAR(XPLT        ,YPLT        ,CSP,'C'  ,0.0,1)
       CALL PLCHAR(XPLT+0.9*CSP,YPLT-0.4*CSP,CSC, 'Do',0.0,2)
C
      ELSEIF(IP.EQ.IPTHE ) THEN
ccc       CALL PLMATH(XPLT,YPLT,CSP,'Q"' ,0.0,2)
       CALL PLCHAR(XPLT,YPLT,CSP,'elev' ,0.0,4)
C
      ELSEIF(IP.EQ.IPPHI ) THEN
ccc       CALL PLMATH(XPLT,YPLT,CSP,'F"' ,0.0,2)
       CALL PLCHAR(XPLT,YPLT,CSP,'bank' ,0.0,4)
C
      ELSEIF(IP.EQ.IPVEE ) THEN
       CALL PLCHAR(XPLT,YPLT,CSP,'V'  ,0.0,1)
C
      ELSEIF(IP.EQ.IPRHO ) THEN
       CALL PLMATH(XPLT,YPLT,CSP,'r'  ,0.0,1)
C
      ELSEIF(IP.EQ.IPGEE ) THEN
       CALL PLCHAR(XPLT,YPLT,CSP,'g'  ,0.0,1)
C
      ELSEIF(IP.EQ.IPRAD ) THEN
       CALL PLCHAR(XPLT        ,YPLT        ,CSP,'R'   ,0.0,1)
       CALL PLCHAR(XPLT+0.9*CSP,YPLT-0.4*CSP,CSC,'turn',0.0,4)

C
      ELSEIF(IP.EQ.IPFAC ) THEN
       CALL PLCHAR(XPLT,YPLT,CSP,'N'  ,0.0,1)
C
      ELSEIF(IP.EQ.IPXCG ) THEN
       CALL PLCHAR(XPLT        ,YPLT        ,CSP,'X'  ,0.0,1)
       CALL PLCHAR(XPLT+0.9*CSP,YPLT-0.4*CSP,CSC, 'cg',0.0,2)
C
      ELSEIF(IP.EQ.IPYCG ) THEN
       CALL PLCHAR(XPLT        ,YPLT        ,CSP,'Y'  ,0.0,1)
       CALL PLCHAR(XPLT+0.9*CSP,YPLT-0.4*CSP,CSC, 'cg',0.0,2)
C
      ELSEIF(IP.EQ.IPZCG ) THEN
       CALL PLCHAR(XPLT        ,YPLT        ,CSP,'Z'  ,0.0,1)
       CALL PLCHAR(XPLT+0.9*CSP,YPLT-0.4*CSP,CSC, 'cg',0.0,2)
C
      ELSEIF(IP.EQ.IPMASS) THEN
       CALL PLCHAR(XPLT        ,YPLT        ,CSP,'mass',0.0,4)
C
      ELSEIF(IP.EQ.IPIXX ) THEN
       CALL PLCHAR(XPLT        ,YPLT        ,CSP,'I'  ,0.0,1)
       CALL PLCHAR(XPLT+0.9*CSP,YPLT-0.4*CSP,CSC, 'xx',0.0,2)
C
      ELSEIF(IP.EQ.IPIYY ) THEN
       CALL PLCHAR(XPLT        ,YPLT        ,CSP,'I'  ,0.0,1)
       CALL PLCHAR(XPLT+0.9*CSP,YPLT-0.4*CSP,CSC, 'yy',0.0,2)
C
      ELSEIF(IP.EQ.IPIZZ ) THEN
       CALL PLCHAR(XPLT        ,YPLT        ,CSP,'I'  ,0.0,1)
       CALL PLCHAR(XPLT+0.9*CSP,YPLT-0.4*CSP,CSC, 'zz',0.0,2)
C
      ELSEIF(IP.EQ.IPIXY ) THEN
       CALL PLCHAR(XPLT        ,YPLT        ,CSP,'I'  ,0.0,1)
       CALL PLCHAR(XPLT+0.9*CSP,YPLT-0.4*CSP,CSC, 'xy',0.0,2)
C
      ELSEIF(IP.EQ.IPIYZ ) THEN
       CALL PLCHAR(XPLT        ,YPLT        ,CSP,'I'  ,0.0,1)
       CALL PLCHAR(XPLT+0.9*CSP,YPLT-0.4*CSP,CSC, 'yz',0.0,2)
C
      ELSEIF(IP.EQ.IPIZX ) THEN
       CALL PLCHAR(XPLT        ,YPLT        ,CSP,'I'  ,0.0,1)
       CALL PLCHAR(XPLT+0.9*CSP,YPLT-0.4*CSP,CSC, 'zx',0.0,2)
C
      ELSEIF(IP.EQ.IPCLA ) THEN
       CALL PLMATH(XPLT        ,YPLT        ,CSP,'D'  ,0.0,1)
       CALL PLCHAR(XPLT+1.0*CSP,YPLT        ,CSP,'C'  ,0.0,1)
       CALL PLCHAR(XPLT+1.9*CSP,YPLT-0.4*CSP,CSC, 'L' ,0.0,1)
       CALL PLMATH(XPLT+2.7*CSP,YPLT-0.6*CSP,CSC,  'a',0.0,1)
C
      ELSEIF(IP.EQ.IPCLU ) THEN
       CALL PLMATH(XPLT        ,YPLT        ,CSP,'D'  ,0.0,1)
       CALL PLCHAR(XPLT+1.0*CSP,YPLT        ,CSP,'C'  ,0.0,1)
       CALL PLCHAR(XPLT+1.9*CSP,YPLT-0.4*CSP,CSC, 'L' ,0.0,1)
       CALL PLCHAR(XPLT+2.7*CSP,YPLT-0.6*CSP,CSC,  'u',0.0,1)
C
      ELSEIF(IP.EQ.IPCMA ) THEN
       CALL PLMATH(XPLT        ,YPLT        ,CSP,'D'  ,0.0,1)
       CALL PLCHAR(XPLT+1.0*CSP,YPLT        ,CSP,'C'  ,0.0,1)
       CALL PLCHAR(XPLT+1.9*CSP,YPLT-0.4*CSP,CSC, 'M' ,0.0,1)
       CALL PLMATH(XPLT+2.7*CSP,YPLT-0.6*CSP,CSC,  'a',0.0,1)
C
      ELSEIF(IP.EQ.IPCMU ) THEN
       CALL PLMATH(XPLT        ,YPLT        ,CSP,'D'  ,0.0,1)
       CALL PLCHAR(XPLT+1.0*CSP,YPLT        ,CSP,'C'  ,0.0,1)
       CALL PLCHAR(XPLT+1.9*CSP,YPLT-0.4*CSP,CSC, 'M' ,0.0,1)
       CALL PLCHAR(XPLT+2.7*CSP,YPLT-0.6*CSP,CSC,  'u',0.0,1)
C
      ENDIF
C
C---- make sure subscript didn't screw up vertical pen location
      CALL PLCHAR(999.0,YPLT,0.001*CSP,' ',0.0,1)
C
      RETURN
      END ! PPNAME


      SUBROUTINE PLNUMS(X,Y,CS,RNUM,ANG,NDIG)
C----------------------------------------------------------
C     Plots real number with NDIG significant digits
C----------------------------------------------------------
C
      IF(RNUM.EQ.0.0) THEN
       XPLT = X + CS*FLOAT(NDIG/2-1)
       CALL PLNUMB(XPLT,Y,CS,RNUM,ANG,1)
       RETURN
      ENDIF
C
      ARNUM = ABS(RNUM)
      RLOG = LOG10(ARNUM)
C
C---- set number of digits to left and right of decimal point
      NL = INT( RLOG + 101.0 ) - 100
      NR = NDIG - NL
C
      IF(NL.GT.NDIG .OR. NR.GT.NDIG) THEN
C----- plot with scientific notation
       NEXP = 1 - NL
       FEXP = 10.0**NEXP
       ND = MAX( NDIG - 1 , 1 )
       CALL PLNUMB(X   ,Y,CS,RNUM*FEXP,ANG,ND)
       CALL PLCHAR(999.,Y,CS,'e',ANG,1)
       CALL PLNUMB(999.,Y,CS,FLOAT(-NEXP),ANG,-1)
      ELSE
       ND = MAX( NR , 0 )
       CALL PLNUMB(X,Y,CS,RNUM,ANG,ND)
      ENDIF
C
      RETURN
      END
