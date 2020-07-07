C
C----- CIRCLE.INC include file for circle-plane operations
C
C                                                              n
C ICX   number of circle-plane points for complex mapping   ( 2  + 1 )
C IMX   number of complex mapping coefficients  Cn

      PARAMETER (ICX=257)
      PARAMETER (IMX=(ICX-1)/4)
C
      COMPLEX ZCOLDW, DZTE, CHORDZ, ZLEOLD, ZC, ZC_CN, PIQ, CN, EIW
C
      COMMON/CPI01/ NC,MC,MCT
C
      COMMON/CPR01/ PI,AGTE,AG0,QIM0,QIMOLD,
     &              DWC,WC(ICX),SC(ICX), 
     &              SCOLD(ICX),XCOLD(ICX),YCOLD(ICX)
C
      COMMON/CPC01/ DZTE, CHORDZ, ZLEOLD, ZCOLDW(ICX), 
     &              ZC(ICX), ZC_CN(ICX,IMX/4),
     &              PIQ(ICX), CN(0:IMX), EIW(ICX,0:IMX)
C

C   NC         number of circle plane points, must be 2**n + 1
C   MC         number of Fourier harmonics of P(w) + iQ(w)
C   MCT        number of Fourier harmonics for which dZC/dCN are calculated
C
C   PI         3.1415926
C   AGTE       trailing edge angle/pi
C   AG0        angle of airfoil surface at first point
C   QIM0       Q(w) offset   = Q(0)
C   QIMOLD     Q(w) offset for old airfoil
C   DWC        increment of circle-plane coordinate w,  DWC = 2 pi/(NC-1)
C   WC(.)      circle plane coordinate w for Fourier operations
C   SC(.)      normalized arc length array s(w)
C   SCOLD(.)   normalized arc length s(w) of old airfoil
C   XCOLD(.)   x coordinate x(w) of old airfoil
C   YCOLD(.)   y coordinate y(w) of old airfoil
C
C   DZTE       trailing edge gap specified in the complex plane
C   CHORDZ     airfoil chord specified in the complex plane
C   ZLEOLD     leading edge of old airfoil
C   ZCOLDW(.)  d(x+iy)/dw of old airfoil
C   ZC(.)      complex airfoil coordinates derived from P(w) + iQ(w)
C   ZC_CN(..)  sensitivities dZC/dCN for driving geometry constraints
C   PIQ(.)     complex harmonic function P(w) + iQ(w)
C   CN(.)      Fourier coefficients of P(w) + iQ(w)
C   EIW(..)    complex number  exp(inw)  array on the unit circle 
