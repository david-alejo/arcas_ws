/*
 * File: brazo_ros.cpp
 *
 * Real-Time Workshop code generated for Simulink model brazo_ros.
 *
 * Model version                        : 1.28
 * Real-Time Workshop file version      : 7.6  (R2010b)  03-Aug-2010
 * Real-Time Workshop file generated on : Mon Jun 24 11:16:39 2013
 * TLC version                          : 7.6 (Jul 13 2010)
 * C/C++ source code generated on       : Mon Jun 24 11:16:40 2013
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Generic->32-bit x86 compatible
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "brazo_ros.h"
#include "brazo_ros_private.h"

/* Block states (auto storage) */
D_Work_brazo_ros brazo_ros_DWork;

/* External inputs (root inport signals with auto storage) */
ExternalInputs_brazo_ros brazo_ros_U;

/* External outputs (root outports fed by signals with auto storage) */
ExternalOutputs_brazo_ros brazo_ros_Y;

/* Real-time model */
RT_MODEL_brazo_ros brazo_ros_M_;
RT_MODEL_brazo_ros *brazo_ros_M = &brazo_ros_M_;
real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    return (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    return atan2(u0 > 0.0 ? 1.0 : -1.0, u1 > 0.0 ? 1.0 : -1.0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      return RT_PI / 2.0;
    } else if (u0 < 0.0) {
      return -(RT_PI / 2.0);
    } else {
      return 0.0;
    }
  } else {
    return atan2(u0, u1);
  }
}

/* Model step function */
void brazo_ros_step(void)
{
  real_T q;
  real_T q_0;
  real_T q_1;
  real_T q_2;
  real_T q_3;
  real_T Pm_x;
  real_T Pm_y;
  real_T Pm_z;
  real_T dPm_x;
  real_T dPm_y;
  real_T r_ext;
  real_T beta;
  boolean_T qX_b;
  boolean_T guard;

  /* Embedded MATLAB: '<Root>/Embedded MATLAB Function' incorporates:
   *  Constant: '<Root>/Constant1'
   *  Constant: '<Root>/Constant2'
   *  Constant: '<Root>/Constant3'
   *  Inport: '<Root>/P_X'
   *  Inport: '<Root>/P_X1'
   *  Inport: '<Root>/P_X2'
   */
  /* Embedded MATLAB Function 'Embedded MATLAB Function': '<S1>:1' */
  /* Inicializacion de variables */
  /* '<S1>:1:30' */
  q = brazo_ros_DWork.q1_1;

  /* '<S1>:1:31' */
  q_0 = brazo_ros_DWork.q2_1;

  /* '<S1>:1:32' */
  q_1 = brazo_ros_DWork.q3_1;

  /* '<S1>:1:33' */
  q_2 = brazo_ros_DWork.q4_1;

  /* '<S1>:1:34' */
  q_3 = brazo_ros_DWork.q5_1;

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /* Longitudes de los tramos del brazo (en metros) */
  /* '<S1>:1:39' */
  /* '<S1>:1:40' */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /* Limites de las variables articulares */
  /* '<S1>:1:47' */
  /* '<S1>:1:48' */
  /* '<S1>:1:49' */
  /* Se reduce el intervalo */
  /* '<S1>:1:50' */
  /* '<S1>:1:51' */
  /* '<S1>:1:52' */
  /* Limite de seguridad para evitar autocolisión */
  /* '<S1>:1:53' */
  /* '<S1>:1:54' */
  /* '<S1>:1:55' */
  /* '<S1>:1:56' */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Posicion de la muñeca */
  /* '<S1>:1:61' */
  Pm_x = brazo_ros_U.P_X - 0.1113 * brazo_ros_P.Constant1_Value;

  /* '<S1>:1:62' */
  Pm_y = brazo_ros_U.P_X1 - 0.1113 * brazo_ros_P.Constant2_Value;

  /* '<S1>:1:63' */
  Pm_z = brazo_ros_U.P_X2 - 0.1113 * brazo_ros_P.Constant3_Value;

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Calculo de Q1, Q2 y Q3  */
  /* '<S1>:1:68' */
  brazo_ros_DWork.q1_1 = Pm_y / Pm_x;
  brazo_ros_DWork.q1_1 = atan(brazo_ros_DWork.q1_1);

  /* '<S1>:1:70' */
  dPm_x = 0.007 * cos(brazo_ros_DWork.q1_1);

  /* '<S1>:1:71' */
  dPm_y = 0.007 * sin(brazo_ros_DWork.q1_1);

  /* '<S1>:1:73' */
  r_ext = sqrt(rt_pow_snf(Pm_x + dPm_x, 2.0) + rt_pow_snf(Pm_y + dPm_y, 2.0));
  if (Pm_x > -dPm_x) {
    /* '<S1>:1:75' */
    /* '<S1>:1:76' */
    beta = rt_atan2d_snf(Pm_z, r_ext);
  } else {
    /* '<S1>:1:78' */
    beta = rt_atan2d_snf(Pm_z, -r_ext);
  }

  /* '<S1>:1:81' */
  r_ext = ((((rt_pow_snf(Pm_x + dPm_x, 2.0) + rt_pow_snf(Pm_y + dPm_y, 2.0)) +
             rt_pow_snf(Pm_z, 2.0)) - 0.06091024) - 0.01221025) /
    0.054542799999999995;

  /* '<S1>:1:83' */
  brazo_ros_DWork.q3_1 = rt_atan2d_snf(sqrt(1.00000000000001 - rt_pow_snf(r_ext,
    2.0)), r_ext);

  /* '<S1>:1:85' */
  /* '<S1>:1:87' */
  brazo_ros_DWork.q2_1 = beta - atan(0.1105 * sin(brazo_ros_DWork.q3_1) /
    (0.1105 * cos(brazo_ros_DWork.q3_1) + 0.2468));

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Calculo de Q4A  */
  /* '<S1>:1:92' */
  r_ext = rt_atan2d_snf(sin(brazo_ros_DWork.q1_1) * brazo_ros_P.Constant1_Value
                        - cos(brazo_ros_DWork.q1_1) *
                        brazo_ros_P.Constant2_Value, (cos((brazo_ros_DWork.q3_1
    + 1.5707963267948966) + brazo_ros_DWork.q2_1) * cos(brazo_ros_DWork.q1_1) *
    brazo_ros_P.Constant1_Value + cos((brazo_ros_DWork.q3_1 + 1.5707963267948966)
    + brazo_ros_DWork.q2_1) * sin(brazo_ros_DWork.q1_1) *
    brazo_ros_P.Constant2_Value) + sin((brazo_ros_DWork.q2_1 +
    brazo_ros_DWork.q3_1) + 1.5707963267948966) * brazo_ros_P.Constant3_Value);
  if (!((r_ext > -1.5707963267948966) && (r_ext < 1.5707963267948966))) {
    if (r_ext < -1.5707963267948966) {
      /* '<S1>:1:97' */
      /* '<S1>:1:98' */
      r_ext += 3.1415926535897931;
    } else {
      if (r_ext > 1.5707963267948966) {
        /* '<S1>:1:99' */
        /* '<S1>:1:100' */
        r_ext -= 3.1415926535897931;
      }
    }
  } else {
    /* '<S1>:1:94' */
  }

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Calculo de Q5A  */
  /* '<S1>:1:107' */
  Pm_x = acos((sin((brazo_ros_DWork.q3_1 + 1.5707963267948966) +
                   brazo_ros_DWork.q2_1) * cos(brazo_ros_DWork.q1_1) *
               brazo_ros_P.Constant1_Value + sin((brazo_ros_DWork.q3_1 +
    1.5707963267948966) + brazo_ros_DWork.q2_1) * sin(brazo_ros_DWork.q1_1) *
               brazo_ros_P.Constant2_Value) - cos((brazo_ros_DWork.q2_1 +
    brazo_ros_DWork.q3_1) + 1.5707963267948966) * brazo_ros_P.Constant3_Value);

  /* '<S1>:1:108' */
  /* '<S1>:1:110' */
  /* '<S1>:1:112' */
  if (!((fabs((((((cos((brazo_ros_DWork.q2_1 + brazo_ros_DWork.q3_1) +
                       1.5707963267948966) * cos(r_ext) * sin(Pm_x) + sin
                   ((brazo_ros_DWork.q2_1 + brazo_ros_DWork.q3_1) +
                    1.5707963267948966) * cos(Pm_x)) * 0.1113 + sin
                  ((brazo_ros_DWork.q2_1 + brazo_ros_DWork.q3_1) +
                   1.5707963267948966) * 0.1105) + cos(brazo_ros_DWork.q2_1) *
                 0.2468) - 0.007) * cos(brazo_ros_DWork.q1_1) + sin
               (brazo_ros_DWork.q1_1) * sin(r_ext) * sin(Pm_x) * 0.1113) -
              brazo_ros_U.P_X) + fabs((((((cos((brazo_ros_DWork.q2_1 +
                brazo_ros_DWork.q3_1) + 1.5707963267948966) * cos(r_ext) * sin
              (Pm_x) + sin((brazo_ros_DWork.q2_1 + brazo_ros_DWork.q3_1) +
               1.5707963267948966) * cos(Pm_x)) * 0.1113 + sin
             ((brazo_ros_DWork.q2_1 + brazo_ros_DWork.q3_1) + 1.5707963267948966)
             * 0.1105) + cos(brazo_ros_DWork.q2_1) * 0.2468) - 0.007) * sin
          (brazo_ros_DWork.q1_1) + (-cos(brazo_ros_DWork.q1_1)) * sin(r_ext) *
          sin(Pm_x) * 0.1113) - brazo_ros_U.P_X1)) + fabs((((sin
           ((brazo_ros_DWork.q2_1 + brazo_ros_DWork.q3_1) + 1.5707963267948966) *
           cos(r_ext) * sin(Pm_x) - cos((brazo_ros_DWork.q2_1 +
             brazo_ros_DWork.q3_1) + 1.5707963267948966) * cos(Pm_x)) * 0.1113 -
          cos((brazo_ros_DWork.q2_1 + brazo_ros_DWork.q3_1) + 1.5707963267948966)
          * 0.1105) + sin(brazo_ros_DWork.q2_1) * 0.2468) - brazo_ros_U.P_X2) <
        1.0E-5)) {
    /* '<S1>:1:118' */
    Pm_x = -Pm_x;
  } else {
    /* '<S1>:1:115' */
    /* Precision de 0.01mm */
    /* '<S1>:1:116' */
  }

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Calculo de Q4B  */
  /* '<S1>:1:124' */
  qX_b = TRUE;
  if (r_ext - 3.1415926535897931 >= -1.5707963267948966) {
    /* '<S1>:1:125' */
    /* '<S1>:1:126' */
    Pm_y = r_ext - 3.1415926535897931;
  } else if (r_ext + 3.1415926535897931 <= 1.5707963267948966) {
    /* '<S1>:1:127' */
    /* '<S1>:1:128' */
    Pm_y = r_ext + 3.1415926535897931;
  } else {
    /* '<S1>:1:130' */
    Pm_y = 0.0;

    /* SIMULINK al compilar exige dar algun valor aunque no se use */
    /* '<S1>:1:131' */
    qX_b = FALSE;
  }

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Calculo de Q5B  */
  /* '<S1>:1:137' */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Eleccion entre solución A o B para Q4 y Q5 */
  /*  (si existe la solución B, el criterio de seleccion entre las soluciones  */
  /*  A y B se basa solo en el valor de Q4: se elige el valor de Q4_X que más  */
  /*  cercano este al valor actual de Q4)  */
  if ((int32_T)qX_b == 0) {
    /* '<S1>:1:145' */
    if (Pm_x == 0.0) {
      /* '<S1>:1:146' */
      /* '<S1>:1:147' */
      brazo_ros_DWork.q5_1 = 0.0;

      /* q4_1 no cambia */
    } else {
      /* '<S1>:1:149' */
      brazo_ros_DWork.q4_1 = r_ext;

      /* '<S1>:1:150' */
      brazo_ros_DWork.q5_1 = Pm_x;
    }
  } else if (Pm_x == 0.0) {
    /* '<S1>:1:153' */
    /* '<S1>:1:154' */
    brazo_ros_DWork.q5_1 = 0.0;

    /* q4_1 no cambia */
  } else {
    /* '<S1>:1:156' */
    /* '<S1>:1:157' */
    if (fabs(brazo_ros_DWork.q4_1 - r_ext) < fabs(brazo_ros_DWork.q4_1 - Pm_y))
    {
      /* '<S1>:1:158' */
      /* '<S1>:1:159' */
      brazo_ros_DWork.q4_1 = r_ext;

      /* '<S1>:1:160' */
      brazo_ros_DWork.q5_1 = Pm_x;
    } else {
      /* '<S1>:1:162' */
      brazo_ros_DWork.q4_1 = Pm_y;

      /* '<S1>:1:163' */
      brazo_ros_DWork.q5_1 = -Pm_x;
    }
  }

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     */
  /*  Comprobacion de rangos */
  /*  (si alguna variable se sale de su rango, el brazo no se mueve a la nueva */
  /*  posicion, es decir, ni siquiera cambian las variables articulares que si  */
  /*  estan dentro de su rango)  */
  if ((brazo_ros_DWork.q1_1 >= -0.87266462599716477) && (brazo_ros_DWork.q1_1 <=
       0.87266462599716477)) {
    /* '<S1>:1:174' */
    if ((brazo_ros_DWork.q2_1 >= -3.1415926535897931) && (brazo_ros_DWork.q2_1 <=
         0.0)) {
      /* '<S1>:1:185' */
      if ((brazo_ros_DWork.q3_1 >= 0.0) && (brazo_ros_DWork.q3_1 <=
           3.1415926535897931)) {
        /* '<S1>:1:196' */
        if ((brazo_ros_DWork.q4_1 >= -1.5707963267948966) &&
            (brazo_ros_DWork.q4_1 <= 1.5707963267948966)) {
          /* '<S1>:1:207' */
          if ((brazo_ros_DWork.q5_1 >= -1.5707963267948966) &&
              (brazo_ros_DWork.q5_1 <= 1.5707963267948966)) {
            /* '<S1>:1:218' */
            /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
            /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     */
            /*  Comprobacion de colisiones */
            /*  (si se detecta alguna colision, el brazo no se mueve a la nueva posicion)  */
            qX_b = FALSE;
            guard = FALSE;
            if ((brazo_ros_DWork.q3_1 >= 1.5707963267948966) &&
                (brazo_ros_DWork.q3_1 <= 2.4434609527920612)) {
              /* '<S1>:1:233' */
              if (brazo_ros_DWork.q5_1 > 2.4434609527920612 -
                  brazo_ros_DWork.q3_1) {
                /* '<S1>:1:234' */
                /* '<S1>:1:235' */
                brazo_ros_DWork.q1_1 = q;

                /* '<S1>:1:236' */
                brazo_ros_DWork.q2_1 = q_0;

                /* '<S1>:1:237' */
                brazo_ros_DWork.q3_1 = q_1;

                /* '<S1>:1:238' */
                brazo_ros_DWork.q4_1 = q_2;

                /* '<S1>:1:239' */
                brazo_ros_DWork.q5_1 = q_3;
              } else if ((brazo_ros_DWork.q5_1 > 0.0) && (brazo_ros_DWork.q2_1 >
                          -(brazo_ros_DWork.q3_1 - 1.5707963267948966))) {
                /* '<S1>:1:242' */
                /* '<S1>:1:243' */
                /* '<S1>:1:244' */
                brazo_ros_DWork.q1_1 = q;

                /* '<S1>:1:245' */
                brazo_ros_DWork.q2_1 = q_0;

                /* '<S1>:1:246' */
                brazo_ros_DWork.q3_1 = q_1;

                /* '<S1>:1:247' */
                brazo_ros_DWork.q4_1 = q_2;

                /* '<S1>:1:248' */
                brazo_ros_DWork.q5_1 = q_3;
              } else {
                guard = TRUE;
              }
            } else {
              guard = TRUE;
            }

            if (guard) {
              if (brazo_ros_DWork.q1_1 > 0.0) {
                /* '<S1>:1:253' */
                if ((brazo_ros_DWork.q4_1 > 0.69813170079773179) &&
                    (brazo_ros_DWork.q5_1 > 0.78539816339744828)) {
                  /* '<S1>:1:254' */
                  /* '<S1>:1:255' */
                  brazo_ros_DWork.q1_1 = q;

                  /* '<S1>:1:256' */
                  brazo_ros_DWork.q2_1 = q_0;

                  /* '<S1>:1:257' */
                  brazo_ros_DWork.q3_1 = q_1;

                  /* '<S1>:1:258' */
                  brazo_ros_DWork.q4_1 = q_2;

                  /* '<S1>:1:259' */
                  brazo_ros_DWork.q5_1 = q_3;
                } else if ((brazo_ros_DWork.q4_1 < -0.69813170079773179) &&
                           (brazo_ros_DWork.q5_1 < -0.78539816339744828)) {
                  /* '<S1>:1:262' */
                  /* '<S1>:1:263' */
                  brazo_ros_DWork.q1_1 = q;

                  /* '<S1>:1:264' */
                  brazo_ros_DWork.q2_1 = q_0;

                  /* '<S1>:1:265' */
                  brazo_ros_DWork.q3_1 = q_1;

                  /* '<S1>:1:266' */
                  brazo_ros_DWork.q4_1 = q_2;

                  /* '<S1>:1:267' */
                  brazo_ros_DWork.q5_1 = q_3;
                } else {
                  qX_b = TRUE;
                }
              } else if ((brazo_ros_DWork.q4_1 < -0.69813170079773179) &&
                         (brazo_ros_DWork.q5_1 > 0.78539816339744828)) {
                /* '<S1>:1:271' */
                /* '<S1>:1:272' */
                brazo_ros_DWork.q1_1 = q;

                /* '<S1>:1:273' */
                brazo_ros_DWork.q2_1 = q_0;

                /* '<S1>:1:274' */
                brazo_ros_DWork.q3_1 = q_1;

                /* '<S1>:1:275' */
                brazo_ros_DWork.q4_1 = q_2;

                /* '<S1>:1:276' */
                brazo_ros_DWork.q5_1 = q_3;
              } else if ((brazo_ros_DWork.q4_1 > 0.69813170079773179) &&
                         (brazo_ros_DWork.q5_1 < -0.78539816339744828)) {
                /* '<S1>:1:279' */
                /* '<S1>:1:280' */
                brazo_ros_DWork.q1_1 = q;

                /* '<S1>:1:281' */
                brazo_ros_DWork.q2_1 = q_0;

                /* '<S1>:1:282' */
                brazo_ros_DWork.q3_1 = q_1;

                /* '<S1>:1:283' */
                brazo_ros_DWork.q4_1 = q_2;

                /* '<S1>:1:284' */
                brazo_ros_DWork.q5_1 = q_3;
              } else {
                qX_b = TRUE;
              }
            }

            if (qX_b) {
              /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
              /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     */
              /*  Asignacion final  */
              /* '<S1>:1:292' */
              q = brazo_ros_DWork.q1_1;

              /* '<S1>:1:293' */
              q_0 = brazo_ros_DWork.q2_1;

              /* '<S1>:1:294' */
              q_1 = brazo_ros_DWork.q3_1;

              /* '<S1>:1:295' */
              q_2 = brazo_ros_DWork.q4_1;

              /* '<S1>:1:296' */
              q_3 = brazo_ros_DWork.q5_1;
            }
          } else {
            /* '<S1>:1:221' */
            brazo_ros_DWork.q1_1 = q;

            /* '<S1>:1:222' */
            brazo_ros_DWork.q2_1 = q_0;

            /* '<S1>:1:223' */
            brazo_ros_DWork.q3_1 = q_1;

            /* '<S1>:1:224' */
            brazo_ros_DWork.q4_1 = q_2;

            /* '<S1>:1:225' */
            brazo_ros_DWork.q5_1 = q_3;
          }
        } else {
          /* '<S1>:1:210' */
          brazo_ros_DWork.q1_1 = q;

          /* '<S1>:1:211' */
          brazo_ros_DWork.q2_1 = q_0;

          /* '<S1>:1:212' */
          brazo_ros_DWork.q3_1 = q_1;

          /* '<S1>:1:213' */
          brazo_ros_DWork.q4_1 = q_2;

          /* '<S1>:1:214' */
          brazo_ros_DWork.q5_1 = q_3;
        }
      } else {
        /* '<S1>:1:199' */
        brazo_ros_DWork.q1_1 = q;

        /* '<S1>:1:200' */
        brazo_ros_DWork.q2_1 = q_0;

        /* '<S1>:1:201' */
        brazo_ros_DWork.q3_1 = q_1;

        /* '<S1>:1:202' */
        brazo_ros_DWork.q4_1 = q_2;

        /* '<S1>:1:203' */
        brazo_ros_DWork.q5_1 = q_3;
      }
    } else {
      /* '<S1>:1:188' */
      brazo_ros_DWork.q1_1 = q;

      /* '<S1>:1:189' */
      brazo_ros_DWork.q2_1 = q_0;

      /* '<S1>:1:190' */
      brazo_ros_DWork.q3_1 = q_1;

      /* '<S1>:1:191' */
      brazo_ros_DWork.q4_1 = q_2;

      /* '<S1>:1:192' */
      brazo_ros_DWork.q5_1 = q_3;
    }
  } else {
    /* '<S1>:1:177' */
    brazo_ros_DWork.q1_1 = q;

    /* '<S1>:1:178' */
    brazo_ros_DWork.q2_1 = q_0;

    /* '<S1>:1:179' */
    brazo_ros_DWork.q3_1 = q_1;

    /* '<S1>:1:180' */
    brazo_ros_DWork.q4_1 = q_2;

    /* '<S1>:1:181' */
    brazo_ros_DWork.q5_1 = q_3;
  }

  /* Outport: '<Root>/Q_1' */
  brazo_ros_Y.Q_1 = q;

  /* Outport: '<Root>/Q_2' */
  brazo_ros_Y.Q_2 = q_0;

  /* Outport: '<Root>/Q_3' */
  brazo_ros_Y.Q_3 = q_1;

  /* Outport: '<Root>/Q_4' */
  brazo_ros_Y.Q_4 = q_2;

  /* Outport: '<Root>/Q_5' */
  brazo_ros_Y.Q_5 = q_3;
}

/* Model initialize function */
void brazo_ros_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(brazo_ros_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&brazo_ros_DWork, 0,
                sizeof(D_Work_brazo_ros));

  /* external inputs */
  (void) memset((void *)&brazo_ros_U, 0,
                sizeof(ExternalInputs_brazo_ros));

  /* external outputs */
  (void) memset((void *)&brazo_ros_Y, 0,
                sizeof(ExternalOutputs_brazo_ros));

  /* InitializeConditions for Embedded MATLAB: '<Root>/Embedded MATLAB Function' */
  brazo_ros_DWork.q1_1 = 0.0;
  brazo_ros_DWork.q2_1 = 0.0;
  brazo_ros_DWork.q3_1 = 0.0;
  brazo_ros_DWork.q4_1 = 0.0;
  brazo_ros_DWork.q5_1 = 0.0;
}

/* Model terminate function */
void brazo_ros_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
