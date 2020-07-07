      SUBROUTINE XYPLOT(N,X,Y,XOFF,XSF,YOFF,YSF,ILIN,SH,ISYM)
      DIMENSION X(N), Y(N)
C
      IF(ISYM.LE.0) CALL XYLINE(N,X,Y,XOFF,XSF,YOFF,YSF,ILIN)
      IF(ISYM.NE.0) CALL XYSYMB(N,X,Y,XOFF,XSF,YOFF,YSF,SH,IABS(ISYM))
C
      RETURN
      END


      SUBROUTINE PLSUBS(XC,YC,CHX,STRING,ANGLE,NC,PLFONT)
C----------------------------------------------------------------
C     Plots character string as a subscript with font routine PLFONT.
C
C      XC,YC  = user coordinates of character to be subscripted
C      CHX    = character width (user coordinates)
C      STRING = subscript character string to plot with NC characters
C      ANGLE  = angle of character (radians, positive is righthanded rotation)
C      NC     = number of subscript characters to plot
C               if NC<0 the length of the string is determined automatically 
C----------------------------------------------------------------
      CHARACTER*(*) STRING
      EXTERNAL PLFONT
      DATA  PI /3.1415926535897932384/
C
C---- subscript character reduction factor, and x,y-shift/chx
      DATA CHFAC, CHDX, CHDY / 0.7, 0.9, -0.4 /
C
      SINA = SIN(ANGLE*PI/180.0)
      COSA = COS(ANGLE*PI/180.0)
C
      X = XC + CHX*(CHDX*COSA - CHDY*SINA)
      Y = YC + CHX*(CHDX*SINA + CHDY*COSA)
      CALL PLFONT(X,Y,CHX*CHFAC,STRING,ANGLE,NC)
C
      RETURN
      END



      SUBROUTINE PLSUPS(XC,YC,CHX,STRING,ANGLE,NC,PLFONT)
C----------------------------------------------------------------
C     Plots character string as a superscript with font routine PLFONT.
C
C      XC,YC  = user coordinates of character to be superscripted
C      CHX    = character width (user coordinates)
C      STRING = superscript character string to plot with NC characters
C      ANGLE  = angle of character (radians, positive is righthanded rotation)
C      NC     = number of superscript characters to plot
C               if NC<0 the length of the string is determined automatically 
C----------------------------------------------------------------
      CHARACTER*(*) STRING
      EXTERNAL PLFONT
      DATA  PI /3.1415926535897932384/
C
C---- superscript character reduction factor, and x,y-shift/chx
      DATA CHFAC, CHDX, CHDY / 0.7, 0.95, 0.7 /
C
      SINA = SIN(ANGLE*PI/180.0)
      COSA = COS(ANGLE*PI/180.0)
C
      X = XC + CHX*(CHDX*COSA - CHDY*SINA)
      Y = YC + CHX*(CHDX*SINA + CHDY*COSA)
      CALL PLFONT(X,Y,CHX*CHFAC,STRING,ANGLE,NC)
C
      RETURN
      END



      SUBROUTINE SCALIT(N,Y,YOFF,YSF,ANN,NANN)
      DIMENSION Y(N)
C.............................................................
C
C     Determines scaling factor for the offset Y array so that
C     YSF*(Ymax-YOFF) will be O(1.0), but less than 1.0.
C
C     ANN = 1.0/YSF is therefore a "nice" plot axis max annotation.
C
C     Y(1:N)   array whose scaling factor is to be determined
C     YOFF     offset of Y array  (Y-YOFF is actually scaled)
C     YSF      Y scaling factor
C     ANN      recommended max Y annotation value  = 1.0/ANN
C     NANN     recommended number of Y annotations
C.............................................................
C
      AG2 = ALOG10(2.0)
      AG5 = ALOG10(5.0)
C
      YMAX = ABS(Y(1) - YOFF)
      DO 10 I=2, N
        YMAX = AMAX1( YMAX , ABS(Y(I)-YOFF) )
   10 CONTINUE
C
      IF(YMAX.EQ.0.0) THEN
       WRITE(*,*) 'SCALIT: Zero array passed in'
       YSF = 1.0E8
       RETURN
      ENDIF
C
      YLOG = ALOG10(YMAX) - 0.001
C
C---- find log of nearest power of 10 above YMAX
      YLOG1 = AINT(YLOG+100.0) - 99.0
 
C---- find log of nearest 2x(power of 10) above YMAX
      YLOG2 = YLOG1 + AG2
      IF(YLOG2-1.0.GT.YLOG) YLOG2 = YLOG2 - 1.0
C
C---- find log of nearest 5x(power of 10) above YMAX
      YLOG5 = YLOG1 + AG5
      IF(YLOG5-1.0.GT.YLOG) YLOG5 = YLOG5 - 1.0
C
C---- find log of smallest upper bound
      GMIN = MIN( YLOG1 , YLOG2 , YLOG5 )
C
      NANN = 5
      IF (GMIN.EQ.YLOG2) NANN = 4
C
C---- set scaling factor and max annotation
      YSF = 10.0**(-GMIN)
      ANN = 1.0/YSF
C
      RETURN
      END ! SCALIT
 


      SUBROUTINE ARROW(X,Y,DX,DY)
C........................................
C
C     Plots arrow from X,Y to X+DX,Y+DY
C........................................
C
C---- fraction of arrow covered by arrowhead, aspect ratio of arrowhead
      DATA FRH, ARH / 0.25, 0.24 /
C
C---- plot arrow
      CALL PLOT(X,Y,3)
      CALL PLOT(X+DX,Y+DY,2)
C
C---- plot arrowhead
      X1 = X + (1.0-FRH)*DX + 0.5*ARH*DY
      Y1 = Y + (1.0-FRH)*DY - 0.5*ARH*DX
      X2 = X + (1.0-FRH)*DX - 0.5*ARH*DY
      Y2 = Y + (1.0-FRH)*DY + 0.5*ARH*DX
      CALL PLOT(X1,Y1,2)
      CALL PLOT(X2,Y2,2)
      CALL PLOT(X+DX,Y+DY,2)
C
      RETURN
      END ! ARROW

