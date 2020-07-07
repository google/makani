C
C---- ROTOR.INC include file
C
      IMPLICIT REAL (M)
C
C--- IX - max number of radial prop stations 
C--- ICASX - max number of stored cases
C--- NPARX - number of case parameters stored
C--- IWX - dimension of work arrays
      PARAMETER ( IX=100, IXP=IX+1, NPARX=12, ICASX=100, IWX=200)
C
C--- NAX - max number of aerodynamic sections defined
C--- NDX - number of aerodynamic parameter defined for each section
      PARAMETER ( NAX=20, NDX=14 )
C
      PARAMETER ( IQ=IX+5, JX=(IQ*3)/2+1 )
      PARAMETER (PI=3.141592654)
      DIMENSION Q(IQ,IQ)
      LOGICAL CONV, GREEK, TERSE, VRTX, FAST, FREE, DUCT, LSTRUC,
     &        WIND, DEST, DESP, STALL, LDESINI, LOPRINI, 
     &        LROTOR, LVNORM, LPWRVAR, LEGEND
C
      REAL MCRIT
C
      LOGICAL LPLOT,LGRID,LLAND,LCURS,LSLOPE
      CHARACTER*80 SAVFIL, FNAME
      CHARACTER*32 NAME
C
      COMMON/UNIT_X/ LUREAD,LUWRIT,LUTEMP,LUSAVE
C
      COMMON/LOGI_X/ CONV,GREEK,TERSE,VRTX,FAST,FREE,DUCT,LSTRUC,
     &               LDESINI,LOPRINI,LROTOR,LVNORM,LPWRVAR,
     &               WIND,DEST,DESP,STALL(IX),LEGEND
C
      COMMON/CHAR_X/ SAVFIL,FNAME,NAME
C
      COMMON/DIME_X/ RHO,RMU,VSO,VEL,RAD,GEE,ALT
C
      COMMON/ICON_X/ II,IINF,INCR,NN,NBLDS,IXSPAC, 
     &               NITERD,NITERA
      COMMON/RCON_X/ VERSION,DT
C
      COMMON/IGEOM_X/ IAERO(IX)
C
      COMMON/RGEOM_X/ CH(IX),BETA(IX),BETA0(IX),T(IX),DBETA,
     &                XI(IX),DXI(IX),XI0,XITIP,XINF,
     &                XPITCH, XV(IX), RAKE
C
      COMMON/IVELS_X/ NADD
      COMMON/RVELS_X/ RADD(IX),
     &                UADD(IX), VADD(IX),
     &                UADDR(IX),VADDR(IX),
     &                UBODY(IX),VBODY(IX),URDUCT
C
      COMMON/AERO_X/ CL(IX),CD(IX),CM(IX),
     &               RE(IX),EFFP(IX),GAM(IX),
     &               DTII(IX), DPII(IX),
     &               DTVI(IX), DPVI(IX),
     &               DTWI(IX), DPWI(IX)
C
      COMMON/IFOIL_X/ NAERO
      COMMON/RFOIL_X/ XIAERO(NAX),AERODATA(NDX,NAX)
C
      COMMON/LOAD_X/ PX(IX),PY(IX),PZ(IX),
     &               MX(IX),MY(IX),MZ(IX),
     &               PX_TY(IX),PX_TZ(IX),PZ_TX(IX),PZ_TY(IX),PZ_WZ(IX),
     &               MY_TY(IX),MZ_TY(IX),MZ_TX(IX),MZ_WZ(IX),
     &               TX(IXP),TY(IXP),TZ(IXP),
     &               WX(IXP),WY(IXP),WZ(IXP),
     &               SHRX(IXP),SHRY(IXP),SHRZ(IXP),
     &               MOMX(IXP),MOMY(IXP),MOMZ(IXP),
     &               EIXXB(IX),EIYYB(IX),
     &               EAB(IX),GJB(IX),EKB(IX),
     &               MB(IX),MXXB(IX),
     &               XOCG(IX),XOSC(IX),
     &               RSTB(IX)
C
      COMMON/RCAS_X/ CASPAR(0:NPARX,ICASX)
      COMMON/ICAS_X/ NCASE,KCASE,IWTYP
C
      COMMON/OPER_X/ ADV,ADW,ADWFCTR,
     &               RMS,RLX,EFFINV,
     &               TSPEC,PSPEC,QSPEC,
     &               TTOT,PTOT,QTOT,
     &               TINV,PINV,TWAK,PWAK,TVIS,PVIS,
     &               GRESMX,FRESMX,ARESMX
C
      COMMON/SENS_X/ TI_ADV,PI_ADV,TI_ADW,PI_ADW,
     &               TW_ADV,PW_ADV,TW_ADW,PW_ADW,
     &               TV_ADV,PV_ADV,TV_ADW,PV_ADW,TV_DBE,PV_DBE,
     &               TI_GAM(IX),PI_GAM(IX),
     &               TW_GAM(IX),PW_GAM(IX),
     &               TV_GAM(IX),PV_GAM(IX)
C
      COMMON/TEMP_X/ W0(IWX),W1(IWX),W2(IWX),W3(IWX),W4(IWX),
     &               W5(IWX),W6(IWX),W7(IWX),W8(IWX),W9(IWX),
     &               T0(IWX),T1(IWX),T2(IWX),T3(IWX),T4(IWX),
     &               T5(IWX),T6(IWX),T7(IWX),T8(IWX),T9(IWX)
C
      COMMON/IPLO_X/ IDEV,IDEVRP,IPSLU,NCOLOR,
     &               LPLOT,LGRID,LLAND,LCURS,LSLOPE
      COMMON/RPLO_X/ SIZE,SCRNFR,PAR,CSIZE,
     &               XWIND,YWIND,
     &               XPAGE,YPAGE,XMARG,YMARG,
     &               XYOFF(2),XYFAC(2)
C
      COMMON/DESI_X/ RADDES,VELDES,ADVDES,RPMDES,R0DES,RWDES,
     &               TDDES,PDDES,
     &               TDES,PDES,CLDES0,CLDES(IX)
C
      COMMON/IPVAR_X/ NPWRVAR
      COMMON/RPVAR_X/ RPMVAR(IX),PWRVAR(IX),XPWRVAR(IX)
C
      COMMON/VIND_X/ VWAK(IX), VW_GAM(IX,IX), VW_ADW(IX), VW_ADV(IX),
     &               VIND(3,IX), VIND_GAM(3,IX,IX), VIND_ADW(3,IX)
C
      COMMON/XWAK_X/ XW0,XWTIP,
     &               XW(IX),  XW_GAM(IX,IX), XW_ADW(IX), XW_ADV(IX),
     &               DXW(IX),DXW_GAM(IX,IX),DXW_ADW(IX),DXW_ADV(IX)
C               
      COMMON/DELT_X/ DGAM(IX), RES(IQ), DADV, DADW, DBET, DEFF, DQ(IQ),
     &               DGAMOLD(IX)
C
ccc   EQUIVALENCE (A_GAMJ(0,1),Q(1,1))
C

C
C   LUREAD       Terminal input  unit number (normally 5)
C   LUWRIT       Terminal output unit number (normally 6)
C   LUTEMP       Disk file unit number
C   LUSAVE       Disk save file output unit number
C       
C   RHO         Fluid density            (dimensioned)
C   RMU         Fluid dynamic viscosity  (dimensioned)
C   VSO         Fluid speed of sound     (dimensioned)
C   VEL         Flight speed             (dimensioned)
C   RAD         Rotor tip radius         (dimensioned)
C   GEE         Earth's acceleration     (dimensioned)
C   ALT         Altitude for fluid properties (km),  999.0 if not defined
C       
C   II          Number of radial stations on blade
C   IINF        Number of radial stations on blade + outer domain
C   INCR        Radial station increment for terminal output
C   NN          Number of perturbation potential harmonic terms
C   NBLDS       Number of blades
C
C   IXSPAC      1 = cosine r/R array stretching
C               2 = sine stretching (less spacing near root)
C   
C   CH(.)       Chord array
C   BETA(.)     Twist angle array
C   BETA0(.)    Static twist angle array (with zero structural loads)
C   T(.)        Dummy radial coordinate array
C   DBETA       Accumulated change in twist angle
C   XI(.)       Radial coordinate array (r/R)
C   DXI(.)      Radial coordinate increment at each station
C   XI0         Blade root radial coordinate value
C   XITIP       Blade tip radial coordinate value  (always = 1)
C   XINF        Outer radial coordinate value where farfield BC is applied
C   XPITCH      x/c location of pitch axis for loads calculations and plots
C   IAERO       Index of inboard aero section for aero characteristics
C       
C   CL(.)       Local lift coefficient array
C   CD(.)       Local drag coefficient array
C   CM(.)       Local blade airfoil Cm
C   GAM(.)      Local circulation array
C   STALL(.)    Local profile stall flag array
C   RE(.)       Local Reynolds number array
C   EFFP(.)     Local profile efficiency array
C   
C-- aero data quantities for each defined radial aerodynamic section
C   NAERO       Number of aerodynamic datasets defined (NAERO>=1)
C   XIAERO      Radial station r/R where aero dataset is defined
C   AERODATA    Aerodynamic definition of the blade section at XIAERO
C               AERODATA( 1,x) = A0 (angle of zero lift)
C               AERODATA( 2,x) = CLMAX (Max CL)
C               AERODATA( 3,x) = CLMIN (Min CL)
C               AERODATA( 4,x) = DCLDA (Incompressible 2-D lift curve slope)
C               AERODATA( 5,x) = DCLDA_STALL (2-D lift curve slope at stall)
C               AERODATA( 6,x) = DCL_STALL (CL increment, onset to full stall)
C               AERODATA( 7,x) = CDMIN (Minimum drag coefficient value)
C               AERODATA( 8,x) = CLDMIN (Lift at minimum drag value)
C               AERODATA( 9,x) = DCDCL2 (Parabolic drag param d(Cd)/dCL^2)
C               AERODATA(10,x) = CMCON (Incompressible 2-D pitching moment)
C               AERODATA(11,x) = REREF (reference Reynold's number)
C               AERODATA(12,x) = REXP (Reynold's number exponent Cd~Re^REXP)
C               AERODATA(13,x) = MCRIT (critical Mach #)
C               AERODATA(14,x) = TOC (thickness/chord)
C
C-- structural quantities below are referred to the following axes:
C-    X-axis points backward along rotation axis
C-    Y-axis points radially outward
C-    Z-axis points sideways at right angles to blade  (Z = X x Y)
C
C   PX PY PZ(.)  load/length in X, Y, Z directions (aero + centrifugal)
C   MX MY MZ(.)  moment/length around X, Y, Z axes (aero + centrifugal)
C
C   PX_TY(.)     sensitivities of loadings to deflections
C   PX_TZ(.)
C   PZ_TX(.)       (A_B  denotes dA/dB)
C   PZ_TY(.)
C   PZ_WZ(.)
C   MY_TY(.)
C   MZ_TY(.)
C   MZ_TX(.)
C   MZ_WZ(.)
C
C   TX TY TZ(.)  deflection angles around X, Y, Z axes
C   WX WY WZ(.)  deflections in X, Y, Z directions
C   SHRX SHRY SHRZ(.)  resultant loads (shear,tension) in X Y Z directions
C   MOMX MOMY MOMZ(.)  resultant moments (b.m., torsion) around X Y Z axes
C
C   EIXXB(.)   bending stiffness in the blade airfoil plane   (dimensioned)
C   EIYYB(.)   bending stiffness out of blade airfoil plane   (dimensioned)
C   EAB(.)     extensional stiffness                          (dimensioned)
C   GJB(.)     torsional stiffness                            (dimensioned)
C   EKB(.)     torsion moment/extension strain  stiffness     (dimensioned)
C   MB(.)      mass/length of blade                           (dimensioned)
C   MXXB(.)    pitch-axis moment of inertia/length of blade   (dimensioned)
C   XOCG(.)    x/c of blade section CG
C   XOSC(.)    x/c of blade section shear center
C   RSTB(.)    radius used for post-processing strain display (dimensioned)
C
C   CASPAR(..) case-parameter array
C          0   case run flags
C          1   advance ratio
C          2   velocity
C          3   tip angle
C          4   altitude
C          5   density
C          6   dynamic viscosity
C          7   speed of sound
C          8   power
C          9   thrust
C         10   torque
C         11   efficiency
C
C   NCASE      current number of saved operating cases
C   KCASE      indicator for independent parameter of case sweep
C            0  = none
C            1  = advance ratio
C            2  = rpm
C            3  = velocity
C            4  = blade angle
C
C   ADV         Advance ratio
C   ADW         Wake advance ratio
C   TPSPEC      Specified thrust, torque, or power
C   RMS, RLX    Rms residual and under-relaxation factor
C   EFFINV      1 / Inviscid efficiency
C   IWTYP       Type of induced velocity model emplyed currently
C                 1 = Graded Momentum,  2 = Potential Formulation
C              
C   TTOT        Rotor inviscid + viscous + nacelle thrust
C   PTOT        Rotor inviscid + viscous + nacelle power
C   QTOT        Rotor inviscid + viscous + nacelle torque  = PTOT*ADV
C   TINV, PINV  Inviscid thrust, power
C   TWAK, PWAK  Inviscid + nacelle thrust, power
C   TVIS, PVIS  Viscous thrust, power
C
C                TTOT  =  TVIS +  TWAK
C                      =  TVIS + (TINV + Tnacelle)
C
C       
C   TI_ADV      Sensitivities of TINV,PINV to advance ratio
C   PI_ADV
C   TI_ADW      Sensitivities of TINV,PINV to wake advance ratio
C   PI_ADW
C   TW_ADV      Sensitivities of TWAK,PWAK to advance ratio
C   PW_ADV
C   TW_ADW      Sensitivities of TWAK,PWAK to wake advance ratio
C   PW_ADW
C   TV_ADV      Sensitivities of TVIS,PVIS to advance ratio
C   PV_ADV
C   TV_ADW      Sensitivities of TVIS,PVIS to wake advance ratio
C   PV_ADW
C   TV_DBE      Sensitivities of TVIS,PVIS to blade angle change
C   PV_DBE
C   TI_GAM(.)   Sensitivity arrays to radial circulation distribution
C   PI_GAM(.)        "
C   TW_GAM(.)        "
C   PW_GAM(.)        "
C   TV_GAM(.)        "
C   PV_GAM(.)        "
C       
C   CONV        T if Converged solution exists
C   GREEK       T if Unrecognized command
C   TERSE       T if Terse output (no radial distributions)
C   FAST        T if Graded Momentum, Potential otherwise
C   FREE        T if Free wake
C   DUCT        T if duct is present
C   LDESINI     T if rotor is to be initialized for design each time
C   LOPRINI     T if rotor is to be initialized for analysis each time
C   LROTOR      T if rotor exists
C   LVNORM      T if flight speed V is used for normalization, wR otherwise
C   WIND        T if windmill-mode plotting is to be used
C
C   DEST        Design-to-thrust option flag
C   DESP        Design-to-power  option flag
C       
C   SAVFIL      Disk output save filename
C   FNAME       Generic filename
C   NAME        Case name
C       
C   W0-9(.)     Temporary work arrays
C   T0-9(.)     Temporary work arrays
C       
C   SIZE        Plot width in inches
C   SCRNFR      Fraction of screen taken up by graphics window
C   PAR         Plot height/width aspect ratio
C   CSIZE       Character size/plot width ratio
C   XWIND,YWIND Dimensions of X-window
C   XPAGE,YPAGE Dimensions of PostScript output page
C   XMARG,YMARG Margins on plot page
C   XYOFF(.)    x,y plot offsets  for current x-y plot, if any
C   XYFAC(.)    x,y scale factors for current x-y plot, if any
C
C   IDEV        Screen window flag
C   IDEVRP      Replotting flag
C   IPSLU       PostScript output-file flag
C   NCOLOR      Number of colors in spectrum
C
C   LPLOT       T if a plot is active
C   LGRID       T if grid is to be overlaid on x,y plots
C   LLAND       T if plots are in Landscape mode
C   LSLOPE      T if slope matching is to be used in cursor-modifications
C
C   RADDES      Design rotor radius             (dimensioned)
C   VELDES      Design speed                    (dimensioned)
C   ADVDES      Design advance ratio            (dimensioned)
C   RPMDES      Design advance ratio            (dimensioned)
C   R0DES       Design root radius              (dimensioned)
C   RWDES       Design disp. body root radius   (dimensioned)
C   TDDES       Design thrust                   (dimensioned)
C   PDDES       Design power                    (dimensioned)
C   TDES        Design thrust
C   PDES        Design power
C   CLDES0      Constant design CL
C   CLDES(.)    Radial design-CL array
C       
C   VTAN(.)     Tangential induced velocity array
C   VWAK(.)     Equivalent-rotor tangential induced velocity array
C   VT_GAM(..)  VTAN-blade bound circulation  sensitivity matrix
C   VT_ADW(.)   VTAN-wake advance ratio sensitivity matrix
C   VW_GAM(..)  VWAK-blade bound circulation  sensitivity matrix
C   VW_ADW(.)   VWAK-wake advance ratio sensitivity matrix
C   VW_ADV(.)   VWAK-advance ratio sensitivity matrix
C   XW0         Root radius of equivalent rotor (= far wake disp. body radius)
C   XWTIP       Tip radius of equivalent rotor
C   XW(.)       Equivalent-rotor radial coordinate array
C   XW_GAM(..)  XW-blade bound circulation  sensitivity matrix
C   XW_ADW(.)   XW-wake advance ratio sensitivity matrix
C   XW_ADV(.)   XW-advance ratio sensitivity matrix
C   DXW(.)      Equivalent-rotor radial coordinate increment array
C   DXW_GAM(..) DXW-blade bound circulation  sensitivity matrix
C   DXW_ADW(.)  DXW-wake advance ratio sensitivity matrix
C   DXW_ADV(.)  DXW-advance ratio sensitivity matrix
C   
C   DGAM(.)     Newton update delta array for bound circulation
C   RES(.)      Newton residual array for bound circulation
C   DADV        Newton update delta for advance ratio
C   DADW        Newton update delta for wake advance ratio
C   DBET        Newton update delta for blade angle
C   DEFF        Newton update delta for 1 / inviscid efficiency
C   DQ(.)       Generic solution vector
C   
C   UBODY(.)    Nacelle perturbation axial  velocity
C   VBODY(.)    Nacelle perturbation radial velocity
C   ABODY(.)    Nacelle cross-sectional area array
C   ZBODY(.)    Nacelle streamwise coordinate
C   NZ          Number of nacelle streamwise stations
    




