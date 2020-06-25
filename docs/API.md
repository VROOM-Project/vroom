<!-- This file is part of VROOM. -->

<!-- Copyright (c) 2015-2018, Julien Coupey. -->
<!-- All rights reserved (see LICENSE). -->

This file describes the `vroom` API.

Contents:
- [Input format](#input)
- [Output format](#output)
- [Examples](#examples)

**Notes**:
- the expected order for all coordinates arrays is `[lon, lat]`
- all timings are in seconds
- all distances are in meters
- a `time_window` object is a pair of timestamps in the form `[start, end]`
- deprecated keys are crossed out
- `cost` values in output are the one used in the optimization objective (currently equal to `duration`)

# Input

The problem description is read from standard input or from a file
(using `-i`) and should be valid `json` formatted as follow.

| Key         | Description |
|-----------|-----------|
| [`jobs`](#jobs) |  array of `job` objects describing the places to visit |
| [`shipments`](#shipments) |  array of `shipment` objects describing pickup and delivery tasks |
| [`vehicles`](#vehicles) |  array of `vehicle` objects describing the available vehicles |
| [[`matrix`](#matrix)] | optional two-dimensional array describing a custom matrix |

## Jobs

A `job` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `id` | an integer used as unique identifier |
| [`description`] | a string describing this job |
| [`location`] | coordinates array |
| [`location_index`] | index of relevant row and column in custom matrix |
| [`service`] | job service duration (defaults to 0) |
| ~~[`amount`]~~ | ~~an array of integers describing multidimensional quantities~~ |
| [`delivery`] | an array of integers describing multidimensional quantities for delivery |
| [`pickup`] | an array of integers describing multidimensional quantities for pickup |
| [`skills`] | an array of integers defining mandatory skills |
| [`priority`] | an integer in the `[0, 100]` range describing priority level (defaults to 0) |
| [`time_windows`] | an array of `time_window` objects describing valid slots for job service start |

## Shipments

A `shipment` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `pickup` | a `shipment_step` object describing pickup |
| `delivery` | a `shipment_step` object describing delivery |
| [`amount`] | an array of integers describing multidimensional quantities |
| [`skills`] | an array of integers defining mandatory skills |
| [`priority`] | an integer in the `[0, 10]` range describing priority level (defaults to 0) |

A `shipment_step` is similar to a `job` object (expect for shared keys already present in `shipment`):

| Key         | Description |
| ----------- | ----------- |
| `id` | an integer used as unique identifier |
| [`description`] | a string describing this step |
| [`location`] | coordinates array |
| [`location_index`] | index of relevant row and column in custom matrix |
| [`service`] | job service duration (defaults to 0) |
| [`time_windows`] | an array of `time_window` objects describing valid slots for job service start |

## Vehicles

A `vehicle` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `id` | an integer used as unique identifier |
| [`profile`] | routing profile (defaults to `car`) |
| [`description`] | a string describing this vehicle |
| [`start`] | coordinates array |
| [`start_index`] | index of relevant row and column in custom matrix |
| [`end`] | coordinates array |
| [`end_index`] | index of relevant row and column in custom matrix |
| [`capacity`] | an array of integers describing multidimensional quantities |
| [`skills`] | an array of integers defining skills |
| [`time_window`] | a `time_window` object describing working hours |
| [`breaks`] | an array of `break` objects |

A `break` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `id` | an integer used as unique identifier |
| `time_windows` | an array of `time_window` objects describing valid slots for break start |
| [`service`] | break duration (defaults to 0) |

## Notes

### Job locations

For `job`, `pickup` and `delivery` objects, if a custom matrix is provided:

- `location_index` is mandatory
- `location` is optional but can be set to retrieve coordinates in the
  response

If no custom matrix is provided:

- a `table` query will be sent to the routing engine
- `location` is mandatory
- `location_index` is irrelevant

### `vehicle` locations

- key `start` and `end` are optional for a `vehicle`, as long as at
  least one of them is present
- if `end` is omitted, the resulting route will stop at the last
  visited job, whose choice is determined by the optimization process
- if `start` is omitted, the resulting route will start at the first
  visited job, whose choice is determined by the optimization process
- to request a round trip, just specify both `start` and `end` with
  the same coordinates
- depending on if a custom matrix is provided, required fields follow
  the same logic than for `job` keys `location` and `location_index`

### Capacity restrictions

Use amounts (`capacity` for vehicles, `delivery` and `pickup` for
jobs, `amount` for shipments) to describe a problem with capacity
restrictions. Those arrays can be used to model custom restrictions
for several metrics at once, e.g. number of items, weight, volume
etc. A vehicle is only allowed to serve a set of jobs if the resulting
load at each route step is lower than the matching value in `capacity`
for each metric. When using multiple components for amounts, it is
recommended to put the most important/limiting metrics first.

It is assumed that all delivery-related amounts for jobs are loaded at
vehicle start, while all pickup-related amounts for jobs are brought
back at vehicle end.

### Skills

Use `skills` to describe a problem where not all jobs can be served by
all vehicles. Job skills are mandatory, i.e. a job can only be served
by a vehicle that has **all** its required skills. In other words:
job `j` is eligible to vehicle `v` iff `j.skills` is included in
`v.skills`.

In order to ease modeling problems with no skills required, it is
assumed that there is no restriction at all if no `skills` keys are
provided.

### Job priorities

Useful in situations where not all jobs can be performed, to gain some
control on which jobs are unassigned. Setting a high `priority` value
for some jobs will tend as much as possible to have them included in
the solution over lower-priority jobs.

### Time windows

It is up to users to decide how to describe time windows:

- **relative values**, e.g. `[0, 14400]` for a 4 hours time window starting at the beginning of the planning horizon. In that case all times reported in output with the `arrival` key are relative to the start of the planning horizon;
- **absolute values**, "real" timestamps. In that case all times reported in output with the `arrival` key can be interpreted as timestamps.

The absence of a time window in input means no timing constraint
applies. In particular, a vehicle with no `time_window` key will be
able to serve any number of jobs, and a job with no `time_windows` key
might be included at any time in any route, to the extent permitted by
other constraints such as skills, capacity and other vehicles/jobs
time windows.

## Matrix

A `matrix` object is an array of arrays of unsigned integers
describing the rows of a custom travel-time matrix as an alternative
to the travel-time matrix computed by the routing engine. Therefore,
if a custom matrix is provided, the `location`, `start` and `end`
properties become optional. Instead of the coordinates, row and column
indications provided with the `*_index` keys are used during
optimization.

# Output

The computed solution is written as `json` on standard output or a file
(using `-o`), formatted as follow.

| Key         | Description |
| ----------- | ----------- |
| `code` | status code |
| `error` | error message (present iff `code` is different from `0`) |
| [`summary`](#summary) | object summarizing solution indicators |
| `unassigned` | array of objects describing unassigned jobs with their `id` and `location` (if provided) |
| [`routes`](#routes) | array of `route` objects |

## Code

Possible values for the status code are:

| Value         | Status |
| ------------- | ----------- |
| `0` | no error raised |
| `1` | internal error |
| `2` | input error |
| `3` | routing error |

## Summary

The `summary` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `cost` | total cost for all routes |
| `unassigned` | number of jobs that could not be served |
| `service` | total service time for all routes |
| `duration` | total travel time for all routes |
| `waiting_time` | total waiting time for all routes |
| ~~[`amount`]~~ | ~~total amount for all routes~~ |
| [`delivery`] | total delivery for all routes |
| [`pickup`] | total pickup for all routes |
| [`distance`]* | total distance for all routes |

*: provided when using the `-g` flag.

## Routes

A `route` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `vehicle` | id of the vehicle assigned to this route |
| [`steps`](#steps) | array of `step` objects |
| `cost` | cost for this route |
| `service` | total service time for this route |
| `duration` | total travel time for this route |
| `waiting_time` | total waiting time for this route |
| ~~[`amount`]~~ | ~~total amount for jobs in this route~~ |
| [`delivery`] | total delivery for jobs in this route |
| [`pickup`] | total pickup for jobs in this route |
| [`description`] | vehicle description, if provided in input |
| [`geometry`]* | polyline encoded route geometry |
| [`distance`]* | total route distance |

*: provided when using the `-g` flag.

### Steps

A `step` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `type` | a string (either `start`, `job`, `pickup`, `delivery`, `break` or `end`) |
| `arrival` | estimated time of arrival at this step |
| `duration` | cumulated travel time upon arrival at this step |
| [`description`] | step description, if provided in input |
| [`location`] | coordinates array for this step (if provided in input) |
| [`id`] | id of the task performed at this step, only provided if `type` value is `job`, `pickup`, `delivery` or `break` |
| ~~[`job`]~~ | ~~id of the job performed at this step, only provided if `type` value is `job`~~ |
| [`load`] | vehicle load after step completion (with capacity constraints) |
| [`service`] | service time at this step (not provided for `start` and `end`) |
| [`waiting_time`] | waiting time upon arrival at this step  (not provided for `start` and `end`) |
| [`distance`]* | traveled distance upon arrival at this step |

*: provided when using the `-g` flag.

# Examples

## Using a routing engine (OSRM or Openrouteservice)

A problem with 2 vehicles, 4 jobs and 1 shipment using capacity,
skills and time window constraints, where matrix computing rely on the
routing engine:

```javascript
{
  "vehicles": [
    {
      "id": 1,
      "start": [
        2.35044,
        48.71764
      ],
      "end": [
        2.35044,
        48.71764
      ],
      "capacity": [
        4
      ],
      "skills": [
        1,
        14
      ],
      "time_window": [
        28800,
        43200
      ],
      "breaks": [
        {
          "id": 1,
          "service": 300,
          "time_windows": [
            [
              32400,
              34200
            ]
          ]
        }
      ]
    },
    {
      "id": 2,
      "start": [
        2.35044,
        48.71764
      ],
      "end": [
        2.35044,
        48.71764
      ],
      "capacity": [
        4
      ],
      "skills": [
        2,
        14
      ],
      "time_window": [
        28800,
        43200
      ],
      "breaks": [
        {
          "id": 2,
          "service": 300,
          "time_windows": [
            [
              34200,
              36000
            ]
          ]
        }
      ]
    }
  ],
  "jobs": [
    {
      "id": 1,
      "service": 300,
      "delivery": [
        1
      ],
      "location": [
        1.98935,
        48.701
      ],
      "skills": [
        1
      ],
      "time_windows": [
        [
          32400,
          36000
        ]
      ]
    },
    {
      "id": 2,
      "service": 300,
      "pickup": [
        1
      ],
      "location": [
        2.03655,
        48.61128
      ],
      "skills": [
        1
      ]
    },
    {
      "id": 5,
      "service": 300,
      "delivery": [
        1
      ],
      "location": [
        2.28325,
        48.5958
      ],
      "skills": [
        14
      ]
    },
    {
      "id": 6,
      "service": 300,
      "delivery": [
        1
      ],
      "location": [
        2.89357,
        48.90736
      ],
      "skills": [
        14
      ]
    }
  ],
  "shipments": [
    {
      "amount": [
        1
      ],
      "skills": [
        2
      ],
      "pickup": {
        "id": 4,
        "service": 300,
        "location": [
          2.41808,
          49.22619
        ]
      },
      "delivery": {
        "id": 3,
        "service": 300,
        "location": [
          2.39719,
          49.07611
        ]
      }
    }
  ]
}
```

producing a solution that looks like:

```javascript
{
  "code": 0,
  "summary": {
    "cost": 20555,
    "unassigned": 0,
    "delivery": [
      4
    ],
    "pickup": [
      2
    ],
    "service": 2400,
    "duration": 20555,
    "waiting_time": 0,
    "distance": 343506
  },
  "unassigned": [],
  "routes": [
    {
      "vehicle": 1,
      "cost": 8359,
      "delivery": [
        2
      ],
      "pickup": [
        1
      ],
      "service": 1200,
      "duration": 8359,
      "waiting_time": 0,
      "distance": 126757,
      "steps": [
        {
          "type": "start",
          "location": [
            2.35044,
            48.71764
          ],
          "load": [
            2
          ],
          "arrival": 28800,
          "duration": 0,
          "distance": 0
        },
        {
          "type": "job",
          "location": [
            2.03655,
            48.61128
          ],
          "id": 2,
          "service": 300,
          "waiting_time": 0,
          "load": [
            3
          ],
          "arrival": 30664,
          "duration": 1864,
          "distance": 30764
        },
        {
          "type": "job",
          "location": [
            2.28325,
            48.5958
          ],
          "id": 5,
          "service": 300,
          "waiting_time": 0,
          "load": [
            2
          ],
          "arrival": 32552,
          "duration": 3452,
          "distance": 52281
        },
        {
          "type": "break",
          "id": 1,
          "service": 300,
          "waiting_time": 0,
          "load": [
            2
          ],
          "arrival": 32852,
          "duration": 3452,
          "distance": 52281
        },
        {
          "type": "job",
          "location": [
            1.98935,
            48.701
          ],
          "id": 1,
          "service": 300,
          "waiting_time": 0,
          "load": [
            1
          ],
          "arrival": 35631,
          "duration": 5931,
          "distance": 89431
        },
        {
          "type": "end",
          "location": [
            2.35044,
            48.71764
          ],
          "load": [
            1
          ],
          "arrival": 38359,
          "duration": 8359,
          "distance": 126757
        }
      ],
      "geometry": "skihH_wiMIbC?^?f@@l@NpEJlCJzCJvCDxA@t@?r@@jBAbBCjBC|@KrCAD[|D]hDYvCOvAk@bFW~BQpBKbBE`B?vBFbBDjADvAAz@E|@Gj@G\\IPKJEJCJAL@H@NBHDH@P@`@AZCj@?RA|@@|@F~BBjCB`YBbN@xABnAFz@L|@Jf@Jf@Tv@lDbKZdAP`APpAFtA@jA?x@ElAChA@|@JlA`@xCt@|FTdBF~BA^AXS\\I\\E`@?XBXH\\JTNPXL^?TMRSL]F]@]LYLWRWRW^a@d@e@f@g@`@St@QpAWdBYh@?b@FZHh@X\\^\\`@P^DTLj@?JIp@Kd@OXw@~AOTy@jA_@p@S\\c@z@]fAMXeAnBMd@E`@AX@^Fj@HZFPDj@?dAShEK~BMnBA^K|AOpCqAtQQpCQfC{@xMKvAWrDkAzQIbAGn@KtAc@bHIdAGd@yA`U_AvNMp@mAdQ_ApKiAdKsAfK_AxGkBjK_AdFc@lBi@dC{BfJqBtIuDhNIReB~EwBvFwAzDOh@Ij@Gn@Cj@@bAFn@Hh@N~@Pn@N^Rd@X^^^`@T^Hh@Fn@D|@DhDL|ADhAJ`@D`@HbARfARpF|AdD~@|Ab@|SdGxErAj@PhCv@pCx@fF|AdEnAfBh@vBr@nAf@rAp@xAx@dAr@`Ax@v@t@t@t@t@x@fAtA`AtAzAfCdAtBpAnCfB~Fv@xCv@vCtBpIbKpa@vAtF`AtDhA`EjB`HjBfGjAxDh@zAvAbEfBbFh@vA~@fC|@xBRh@b@jAlEjK~GnNJz@^t@Hd@n@lBrCbK`AlDXlAHf@Dl@@d@Ah@?r@@d@@vADdAF|@JpARjBXtCr@pHTpBNvAPz@T`Af@fAT\\RX\\XZT\\N\\N~@T`B^vAZ|@^\\P\\Rh@d@d@b@b@h@Zd@`B~BxArB`@p@Pf@Nh@JZXv@DJVj@PX\\n@f@n@RP`Ab@l@`@VXr@fAhCxDb@p@PZp@rAXv@Vn@d@vAj@dBPn@Nl@VjBDr@Dx@@b@A`@?PCJCTC`@APD`@L~BFb@FTFPJPPVTLNPNRHb@l@`KDr@VvEHhBHnBPpGFpB@r@@n@@j@?jB?nCC`BCtA?XGlBGdC?l@AfE@lADjABn@Bx@LxB`@~DLpANtAVxB^rDDdA?dAGtBKtB{@pKIbAEfA?pA@p@F~@Bv@?p@Ch@Gj@I`@GJWLSRQb@Kl@?j@BZH^Xd@XRRNL\\JxAB|@A|AAjAB`AFpA\\zB\\tGB|AHlBH~ALnBTdC\\jD\\nC^nCf@jDJz@x@pGp@jFn@rC^tAHn@AN?NBNDJHHHBHARXnCfGxClGd@~AZtARhARnANjAJnAHnADrABzAAfBEnAKlBQpBW`BWtAa@vAa@nAeAhCyA|CwAhCsAdCQ^MXI\\Kh@Or@GPIJMBMFMJINGPGRANAN@ZBXFVJRJJJHLDL@L?\\NRTLRRb@\\`AVx@Z~Ah@lDv@~DTpAtBzKhA|Fv@pEXtBdAhIzAjMr@`G|ApHVbCAtAA\\ETGXGRITGLKLOHU@QJU\\If@A`@F`@J\\RRJZBLBXBv@Bz@F~@Dt@B`@D\\Jp@Nz@bDxPjDpQzAdI~AdIrDfRpR~`A`Ovu@vBlKd@rAP^h@|@lDjE`D|D|C~Dj@p@x@dAr@z@NTnDlEvAfBvAfB~@hANNNLPNJFHNBH?LBLFJHDH?HEFIDSNOFIJGB?J@FD^\\NH^HVBt@FXDRHLFTNp@h@r@f@vBt@v@Vr@XD@NJPL^`@`@l@\\p@Tl@Hl@Dd@?Z?TE|@Af@@PD^D\\JTDDDFDDHFAL@JBHDHDFFDLLLPr@r@x@^HDbAh@hBr@XNp@^^Zt@r@JHh@^RPb@Tr@l@nBhB~@v@HFtB|ArBzAFNjAz@TP|@n@VJ\\XHFnA`A~AlAp@f@l@b@x@n@xAfApA`AfBrAXTd@f@Zb@zBrDf@~@~AnC|@tAzCfFlGdKXd@PRPVPPPPNJJFRJRFJBHBJ?N?PAHCLELIf@YXKFAL?J@JBHDDBDDFDDHFHDHBHBHBNRdBD`CFrGLdH@`@@\\BVBRDZFRJb@d@`BFXPn@HXf@fBPf@L^P`@NZR`@Vb@JRZb@T\\NTLRjA`B^h@x@jAbCjDlAbBn@|@`@l@`@j@Xb@V^Xf@NX\\t@p@|Ap@zAn@zA`@jAZ|@Vx@t@~BRl@L\\`@~@Rb@Xj@RZXh@NTb@n@Zb@d@j@NPXXTVb@`@TPd@\\XTXPXPXN^RVLXJXJ`@N^Jz@VjAb@^Pz@VP?ZJTHPDb@JdAZh@Nl@Ph@Pb@L`@Tn@\\^Tj@`@LJt@x@f@d@Z^VVl@v@PXDSBGDKNe@L_@Ty@|AuFdCqJd@iBtAsF??r@kD|@qDz@{Cx@iC~@iBHQjH}MbByCv@iBj@sAtByDp@sAVa@HOFK^e@HKVUfA_AlAsALObBkAXQ|BuAb@Yn@e@LIhA{@fB{ArAgAvAsAxBmBVUVUzM{NROzAiBjCmCbDkDRS`B_BTS|@{@bBcBLKRW|EmEtAgBtAiBtAgBJMt@iAXY`AeAfAaAhAmAHMBDFDD@D?D?BABC@G@ICIAE\\UXM^MRGv@UxBk@LEFCDC@CBE?GHcBTeCHkA@kAC{@E}@q@aEe@gBI_@E]CkACw@IiEAw@B[@ODKf@u@^c@POPGXKtASx@IjDUpAK`@Ib@Mb@Wb@]f@u@P]Ne@dBeGfByFx@sBv@wBp@gCn@gC|A_Gn@aCp@oBdCaG~@_CZ_ATgAlBuI~AwEjBeEx@kCvAmGpBgF~CcI~CiGBEvEaItAaC~AuE^s@`@o@tAcBT_@Pa@z@mCfAeCDM?WGeD?_A@ODc@RFlAT?w@B_ABm@Bi@HuBBy@@_AAq@KyAEa@Os@YgBI{@@mABeADe@RaBnAgEv@wDNmAJuAH{AByACkBIcCYgEIyACqA@qA@mFA{@Au@CaA]aFy@iMkAsRq@yI[kESuBQwCQwB[yKu@_WIwOGeK?}@?e@Ag@Ck@E_A@_@JY@YGYCa@Cw@A{A?e@@g@FmA@e@?WA_@Ac@Ea@Gm@Gy@Iq@S{B[eCYkBWqAWiA_@cB]{A]_B_@uBQq@Sg@Ug@k@iA{@kBcCcFk@iAc@cAUi@Qo@Os@Ks@Ky@IoAKkCKaDSqGe@sNa@{K[}Ja@cMEiA]sJC[EkAAu@Au@Aw@B{D?uB?eA?oAEuAEmAGsAIy@OaBQqAUaBSuAoA}HKiABg@Ha@HGHMJ[@a@E[CIGOOOKa@[s@kAkDaAiEYkBUkBSsCKmDGmFAwAE_BIgAWwCOwAQgAUiAk@_Ce@cBs@yBwBoGgB_Gm@_Ci@oCYkBUcBYsCe@cG[mEa@mEc@eEYsB]wBa@qB]yAi@wBu@oCgCuImFkQ]uAk@aCEY?YDU@]E]IWMMMEKAK@OGKSIa@Mq@Uw@Uu@Yk@mCaJgCqIg@cBe@}Aa@cBa@kB_@sB]gCOcBM{AIsBIqBIyDK_GSgJHmBBsAB{@Bk@Fs@Fi@H_@NIJSBUAWESKMGuAKoBKyBIy@My@]}Ai@eC]uAi@{B_AeEg@qDu@kGM{@[qAUq@[m@e@m@OQYS]Oi@O[Gm@AsC@[I_Aa@_@s@Se@E[IYKUGo@?WBc@Fi@n@sJVuDDu@`@eGf@aHT}CFs@Hm@L_AN}@Ps@Nk@Xo@Xe@\\?ZKTYPc@Dg@?m@Ig@F]DQHMLGTCv@CXCJC\\I@C@C?C?w@?W?cA?sB?eA@YBSFUl@yBdAuD?W?OAOEMEMe@k@g@q@MQc@k@EESYCQCSAW?W?_@@YBSDSXiAHa@Lu@BYHy@@_@@g@@UDQFQHMTYvBoCT]bAoAZk@RNNSz@}AJQR[FMFODKBM@M?MAMAQGc@Gi@E_@E[Ok@CQC[AWAOAS?Q?i@@c@AWAa@Cc@Ca@Ga@Q_A?M@O@KDKFGZYRSNQRYRSXm@La@lAkCRc@~@uBRk@WWaD_DM]@Y@I@KAKAIEGEEEAc@k@Qc@I]w@sEKy@Ew@GeAA[AWA]W{DCQAk@BqCDqD?k@Ba@FcDAg@@i@BoAFoA@e@@_@DQD?DCBI@IAGCGECG@CSAKKcAAMEYAm@EuAAoBCcAKgB]cD]sDQoAY_BYwAEQa@mBQo@Qm@IYG]CM?ECUAQAIA[?[A[@s@As@As@A[Cg@EWEQFEDI@KAK??EIGEG?Cu@AgAGwBGcBGgAGi@E]Km@Og@Us@a@mAUo@Mk@Ia@UsAGa@Mm@K_@ESGe@Ea@WyBWcCGi@Km@Ie@Mm@Mk@]aBCMOs@]cBa@kBa@eB]uAa@wAkBmGq@_Cc@cBSy@Oe@o@cCESQ{@WgBGOEMOSKIOIUGOC]KgDy@yD}@}@[UEe@IIAa@IOESGYK[O[MiAg@aAa@q@Ya@QOEKAIEIGy@s@KEUUIGMOWUo@i@aBuAq@k@s@m@[]KKAAAEECCAE?A@A@GAO@IHELYt@a@dASd@O^_DnEa@j@}@nAQVmBhCqAfBaB~Bu@gAkA_Bq@{AQ_@ILW\\OXOZEHCHEXMz@Il@Kj@GV]|@Wl@s@rAwA`CWl@M^OV[f@[b@UTYRo@XQLONOPMP[b@e@~@i@jAMROZM?OEKCS@UKUQOWMe@G[?q@@s@?UASK[GOAACEMQQOIMcAWYIg@KCAo@Sk@Us@Y_Aa@m@e@]g@{@wAW[YU_@S_@Oo@QYI[I[OWU_@WEEKGu@c@IGMQ??BG@O@O?WCWEOCGCGIKGIKEKCKAKBMDIFGHGJELCNCXQj@ST_@VaA`@w@Vm@P_@HM^ETCTAVAV@VBTBTBVJ\\b@bBbAnD\\pAXzARbBN|APrBJ~AHxANZ^zFR~CPxCXzFT|EZdITfIFpCFpCBzCBxC@nC?jCA`DC~CGrDIjDM`DO~CSlDSnCKhAKdA[~CWvBYvBs@xEq@tDg@`Co@zC]xAaAvDeAnD}@hCwA|DaB|Dc@`Ay@jB_AlBgAvBkGvMw@lBw@tBu@rBm@pBg@jBg@rB]zAYxA[bBYjBWfBUlBSjBOdBO`CMbCEfAEhAEpACpA?lAArA?`DBjDFlDJtDJrD\\bJJjDLjDDpBDjCBvE?`ACjBE`CGbBIdBU`DO`BYlC]fCId@Kt@_@|Be@hCg@bC_AlEcArEm@rC_@`B]dBy@fEi@rCc@tC_@dC[fC]pC[pCe@dFc@|Ew@vJ{@rJa@fE]~C_@xCSvAy@nFYvAUjAU`AUhAm@zBQl@a@tAe@tAw@xBkApC]t@c@|@a@t@a@t@SX_@j@W`@_@h@QXSVOTc@j@WZe@f@_DjDgDnDqAvAgAnAoB`Ce@p@i@r@c@n@]h@c@v@gBbDeA`Ci@rAs@fBcBbFwAlEq@vBK\\KVIVITGTGPGRYz@K\\Qd@u@pB]bAa@`Ac@dA]z@a@|@]r@m@hAQZ[j@Yf@Wb@OXU^GHYf@]f@aEhGiEpG}@nAaDpEMV_@h@e@h@i@f@g@`@g@\\w@^s@T]He@Fc@@iAGa@Ea@Ma@Qc@Ue@_@c@e@_AsAyIkMm@{@s@}@oEeF][}@_A_EoEgBcBiAiA_EsDm@m@i@g@SO{@u@cK{HcBqAYYGIa@[e@c@k@w@a@o@g@eAc@kAk@cBUm@W_@UY[YWOc@Qc@Ik@Ai@@g@FcG^eBLyAF_CNs@FqCPkCNUM}@F{@Dy@Hm@De@DG?OAOCOGQOOSSSKEO@MFGHINEVCn@L~BFb@FTFPJPPVTLNPNRHb@l@`KDr@VvEHhBHnBPpGFpB@r@@n@@j@?jB?nCC`BCtA?XGlBGdC?l@AfE@lADjABn@Bx@LxB`@~DLpANtAVxB^rDDdA?dAGtBKtB{@pKIbAEfA?pA@p@F~@Bv@?p@Ch@Gj@I`@GJWLSRQb@Kl@?j@BZH^Xd@XRRNL\\JxAB|@A|AAjAB`AFpA\\zB\\tGB|AHlBH~ALnBTdC\\jD\\nC^nCf@jDJz@x@pGp@jFn@rC^tAHn@AN?NBNDJHHHBHARXnCfGxClGd@~AZtARhARnANjAJnAHnADrABzAAfBEnAKlBQpBW`BWtAa@vAa@nAeAhCyA|CwAhCsAdCQ^MXI\\Kh@Or@GPIJMBMFMJINGPGRANAN@ZBXFVJRJJJHLDL@L?\\NRTLRRb@\\`AVx@Z~Ah@lDv@~DTpAtBzKhA|Fv@pEXtBdAhIzAjMr@`G|ApHVbCAtAA\\ETGXGRITGLKLOHU@QJU\\If@A`@F`@J\\RRJZBLBXBv@Bz@F~@Dt@B`@D\\Jp@Nz@bDxPjDpQzAdI~AdIrDfRpR~`AwMdEOHMNOb@Mf@SnHG`BCpA[jD]dDQ~AcBxJWvAUtAs@jDs@|@a@b@Yh@]xBs@zEMx@Ix@Gx@I~@OxBUrBMf@w@pDk@jFyAlPcAnFcEtLw@fE]rBUrAGfAAfAFr@F^HRLLPHJJHRFVB`@G?IDEJ?NUGWGKC_AKI@I@OBmB`@_@JM@@\\BHBJRh@Vr@Vp@HTDP@TA\\WvBUrBAX?XQL{AdDAD}@vBe@jA[t@k@zAU~@ETW~ACHIb@YvAOv@ShAM~@Gz@I`Be@`HOzBQzBCn@?j@@l@Hx@@V?TCRKXMRa@b@iAnAuBhC_BrBg@v@[n@Sh@Mf@Kt@YnCI^KZOPYPQFWBSAMGIIMSeFyHIMMGGCE?Q?Eh@yAvKc@nCo@pEKr@u@bF_@hCw@rFEr@E|@Ar@MhTCjCBrAd@`MHtB@|AGpAObDg@lH[jFKn@KHWCuAIo@BU\\q@lAs@bAeAd@kAz@iB\\iAxA{AtCY~A{AnEeBfG_AvDoBpHKb@kAlFu@fCw@`Cm@~AG?EBGFCJCN?JINUVk@p@q@x@y@xA]l@]Ze@Xu@Tm@JWHm@d@qB|AiA~@WVGTEJuCaDoA}@gOwJ_Aq@mAeAuB{BqAuBa@}@]eAeGjCg@\\a@n@aFxOg@|AI^EHGBQ@QDg@BIPGTEXCtAF`BA\\G`@KVQVwBbDu@hAIXINOLMFMAOCkHmC[UO_@m@cAIb@Ov@_@`BGv@GbAAVE~AYrOAd@Eb@Gd@qAtKoCxUu@bGa@dD_@nCUzAUnAsCbOq@nDG\\ETCTTfWDtD@zD?`@BZJj@Jh@@ZJLJVLj@Ll@HXH^Vt@Nb@LX|AvCTb@HNRZXXd@N^PVT\\VNJ\\{@HELLJXPIHI??[m@OHINID]z@OK]WWU_@Qe@OYYS[IOUc@}AwCMYOc@Wu@I_@IYMm@Mk@KWKMA[Ki@Kk@C[?a@A{DEuDUgWBUDUF]p@oDrCcOToAT{A^oC`@eDt@cGnCyUpAuKFe@Dc@@e@XsOD_B@WFcAFw@^aBNw@Hc@l@bAN^ZTjHlCNBL@LGNMHOHYt@iAvBcDPWJWFa@@]GaBBuADYFUHQf@CPEPAFCDIH_@f@}A`FyO`@o@f@]dGkC\\dA`@|@pAtBtBzBlAdA~@p@fOvJnA|@tC`DDKFUVWhA_ApB}Al@e@VIl@Kt@Ud@Y\\[\\m@x@yAp@y@j@q@TWHODHDBD?PQBM?QCIEEl@_Bv@aCt@gCjAmFJc@nBqH~@wDdBgGzAoEX_BzAuChAyAhB]jA{@dAe@r@cAp@mAT]n@CtAHVBJIJo@ZkFf@mHNcDFqAA}AIuBe@aMCsABkCLiT@s@D}@Ds@v@sF^iCt@cFJs@n@qEb@oCxAwKDi@P?D?FBLFHLdFxHLRHHLFR@VCPGXQNQJ[H_@XoCJu@Lg@Ri@Zo@f@w@~AsBtBiChAoA`@c@LSJYBS?UAWIy@Am@?k@Bo@P{BN{Bd@aHHaBF{@L_ARiANw@XwAHc@BIV_BDUT_Aj@{AZu@d@kA|@wB@EzAeDPM?Y@YTsBVwB@]AUEQIUWq@Ws@Si@CKCIA]LA^KlBa@NCHAHA~@JJBVFTFBPHHH?HEDK?QCOKICa@GWISKKQIMMISG_@Gs@@gAFgATsA\\sBv@gEbEuLbAoFxAmPj@kFv@qDLg@TsBNyBH_AFy@Hy@Ly@r@{E\\yBXi@`@c@r@}@r@kDTuAVwAbByJP_B\\eDZkDBqAFaBRoHLg@Nc@LONIvMeEqR_aAsDgR_BeI{AeIkDqQcDyPO{@Kq@E]Ca@Eu@?_AD{B@SFWJQPWLa@Be@AYDULUVWZ_@Re@Lg@Ji@Fo@?o@Cm@Cg@Mc@Mc@a@}@gAwBi@iAa@kAYoAIc@[cCoAyKsAiKa@cCu@sEyBcLqAyGc@iCa@cC]{BQy@o@iBY_ASu@CUCKFUDU?W?YEUGUIQEWCU@[@g@Hi@Ni@Vi@`@s@FEb@_AjAyCxA}CdAiC`@oA`@wAVuAVaBPqBJmBDoA@gBC{AEsAIoAKoAOkASoASiA[uAe@_BgAiDmAcC_CsFKw@?OCQEKIIIEK@Ye@YaA_@iBe@aDm@uEEWc@kCWgAg@kD_@oC]oC]kDUeCMoBI_BImBC}AC{GGmCMqB]}DBoD@STSPa@Lo@?o@CWMi@W_@Qe@Ck@D}@J{@Ns@N{@LcAHkABiAJkTAkFEyBk@wRAOQoGO{DEeCCa@IwBCcA]sMGqCI{CEiASiF]gGCg@[uFGiACk@@m@BWDQNUHWFYDg@?W?YMgBCYEc@I[M[KKQKOGMEKIMIKUGUK[Mo@i@kBc@cBe@aBs@}B{@oBe@{@i@_A}CuEmAeB_B}BkAeBKQmAgBc@q@g@s@kCwDiAaBgAkAq@k@w@e@k@U_@OyA]aB_@y@U_@M[Ok@e@Y_@U]Sa@Sk@Qs@Ou@K{@Iw@MuAe@yEMqAOkA]wDMmAIeAG_AAs@?c@?W?[Bo@HkADo@XyCj@cFRe@FKHKNINCJBNDJHHLDNBLBN@RATCTGNEJIHIHMDI@O?IAKEKK{@wAqDgGeAgBk@_A{BeEa@]wD}HoCaG{DcJyBaGg@uAgBcFuAaEg@_BkAsDeBgGmBeHkAkEaAmDsAmFoHgZwEwQ_BmGq@yBaAqCs@gB_AoBo@kA_A_By@mAy@iAo@u@s@w@u@u@y@q@gA}@}@m@w@e@OK_Am@qB{@uAe@eA]aA[gA[qC}@mEqAgDcAaCq@k@QiKyCoR}FkGwBGG}H{CiAc@}@_@k@Yk@ScAe@[QWOYYU_@Wi@Qk@Mw@E}@?y@D{@P_BRy@`DqOHe@pBcHnCyKtD{P`@oBn@eDbB_LlAoHx@qG|AiN|@cK`@oFd@qG|@qMAg@JuBp@qU?g@HeAb@cHJuAFo@HcAjA{QVsDJwAz@yMPgCPqCpAuQNqCJ}AH]TwBZqBVqAd@eBb@oATk@V_@\\Yp@Y|@E@?n@Dj@H`BXlBLf@Bn@Dp@Bt@@v@APQlAGrB[jBe@fBu@pCgBlAcApB}Bd@WvC_Et@}@dDuFb@c@HMBUHu@XwDL}Ai@hAsApCqElJs@tAMf@wAzBoA`B{@bAgAbAqCfBoBx@iBd@yAXwAFs@@iA?IMSAgASkA]uCcAcAe@{@q@c@g@]i@YcAE{@Be@H_@d@iANUv@_BNYXYPKFBF@H?DCBAHIDKBQ@KC[ACEKEEAEKEK?_@QQQa@k@]_@m@a@e@Sq@KqBXwBZeATm@f@e@p@k@p@SJSBQ?QCWWOGQCOYGS[uBUeBu@}Fa@yCKmAA}@BiADmA?y@AkAGuAQqAQaA[eAmDcKUw@Kg@Kg@M}@G{@CoAAyACcNCaYCkCG_CA}@@}@BWB]F[F[JWLKDKDQ?MAOEOEKCUA]?k@@_A@{@EwAEkAGcB?wBDaBJcBPqBV_Cj@cFNwAXwC\\iDZ}D@EJsCB}@BkB@cBAkB?s@Au@EyAKwCK{CKmCOqEAm@?g@?_@HcC"
    },
    {
      "vehicle": 2,
      "cost": 12196,
      "delivery": [
        2
      ],
      "pickup": [
        1
      ],
      "service": 1200,
      "duration": 12196,
      "waiting_time": 0,
      "distance": 216749,
      "steps": [
        {
          "type": "start",
          "location": [
            2.35044,
            48.71764
          ],
          "load": [
            1
          ],
          "arrival": 28800,
          "duration": 0,
          "distance": 0
        },
        {
          "type": "job",
          "location": [
            2.89357,
            48.90736
          ],
          "id": 6,
          "service": 300,
          "waiting_time": 0,
          "load": [
            0
          ],
          "arrival": 31772,
          "duration": 2972,
          "distance": 62693
        },
        {
          "type": "break",
          "id": 2,
          "service": 300,
          "waiting_time": 0,
          "load": [
            0
          ],
          "arrival": 34200,
          "duration": 5100,
          "distance": 100868
        },
        {
          "type": "pickup",
          "location": [
            2.41808,
            49.22619
          ],
          "id": 4,
          "service": 300,
          "waiting_time": 0,
          "load": [
            1
          ],
          "arrival": 36219,
          "duration": 6819,
          "distance": 131706
        },
        {
          "type": "delivery",
          "location": [
            2.39719,
            49.07611
          ],
          "id": 3,
          "service": 300,
          "waiting_time": 0,
          "load": [
            0
          ],
          "arrival": 38400,
          "duration": 8700,
          "distance": 154573
        },
        {
          "type": "end",
          "location": [
            2.35044,
            48.71764
          ],
          "load": [
            0
          ],
          "arrival": 42196,
          "duration": 12196,
          "distance": 216749
        }
      ],
      "geometry": "skihH_wiMIbC?^?f@@l@NpEJlCJzCJvCDxA@t@?r@@jBAbBCjBC|@KrCAD[|D]hDYvCOvAk@bFW~BQpBKbBE`B?vBFbBDjADvAAz@E|@Gj@G\\IPKJEJCJAL@H@NBHDH@P@`@AZCj@?RA|@@|@F~BBjCB`YBbN@xABnAFz@L|@Jf@Jf@Tv@lDbKZdAP`APpAFtA@jA?x@ElAChA@|@JlA`@xCt@|FTdBF~BA^AXS\\I\\E`@?XBXH\\JTNPXLT`@HTBVFv@e@nEo@nFw@~C[tBcAnKGf@Eh@U`B_@rA_@`AsAhCuAjCo@jAg@fAk@xAUl@UC]CqBMsDYsFi@}AIq@EEASEo@EQAeDYgDWgC]iEk@y@OaAY{@]}@a@uAu@MK}@q@q@i@aB}Am@q@k@u@q@{@o@cA[MkFgJqCwE{CiFiEqH_DoFqCeFgIyOy@mB[m@wBcEqA_Co@oAs@sAkAiB{@kAs@}@gAkAs@o@q@k@eAw@oAs@sAq@a@_@}EeCwDsBuAcAq@g@k@g@gC{BgC{BiM{LgB{A}BoBaAw@cBqAqGaFcH_GeHoGcFoEU_@wEoEs@cAg@_Ao@}AoAgEc@mB_@qBOiASaBQqBImBEsBAoBB{AFkBHoAJ}AAi@jAoOz@wLfAyNNeB\\_Fd@_Gn@_JRcEHaCAsARqMB{L@iC?{CWu]QwUCmD@{A@w@DaA`@oLJ{@f@aN?aB@qBKeCUeCMaAQkAk@_COYkAsDCI{@mBg@_Ae@w@eAqAs@s@gAy@WMaAe@eA]iAW{@I{BG}@DM@u@N{@Tw@\\mBhAeRtKwR~KaBdA}@\\s@Nw@Bu@@w@E{@Qc@OqAw@g@e@k@u@i@_Aa@}@a@oA[uAQiAK}@G_AC]mCwb@iBkZeAmMk@uEMgAy@aFoAqG{AqIgAyHq@qF[qEOwFCcI?aDA}AMaCAQOqB}@}EiA_EyCsJwA}E_AmEa@wDKmCAoBXuJ`Ak[By@NuEF_D@{CAsBQcNAm@GcEKyIEwDCgCC_BGgDEyAGmAKsAQyAO_A[wAc@_Bg@sAo@sAeA}A_AaAs@o@q@a@_Aa@}@Wu@OkAMuA@gALcKzAUDsF|@uKv@kHDqBCe@EqEU{Eq@wB_@cCe@uAYSCuA[qDmAiC_A{FmCqBeAy@m@]]yE{CmByAgA_AgCwB_CsBQOkEkDqC}AiCiAeBo@gBi@wBe@m@OoAMoAKqACqAGy@EcJQaBCo@CcFK_CGy@CmADs@Aq@?s@@i@Bs@FyALeBJ_AAoAQ}Am@w@k@s@s@w@eAo@gAi@uAe@}A]oBo@wDKk@e@oBa@wAyAeDo@{AsAyC{BgEWOYq@oDuI{CaHuAoCaA}AqAoAwAgAmAg@_BYuDm@aBa@sAk@mA_AiAmAo@mAy@aBe@yAc@iA]qA_@iBYwAI]{@sEe@mCg@gDq@}F}@cKmBiV}@eM[eDm@{H{Be[u@kMY{KAgHFeH@mABs@N_DR{CXqC^}Cp@eEhA{FzA_Ij@aD`AkGh@mEf@iE^wDXiDLeBNuBp@}Ir@gQv@iWR}FB}@FcDBeDG_EKsCQgCSkBYyB_@qBk@gCo@yBu@sBs@aBcAmByBkDaDkEqB{CaA}A_BmCcB{CcBaDy@_BwBkEeAuBkCwGqBkFGOa@gAeBkFkBqG{@mDYyA[yBWsCSmDEcC?_B?u@BoBBqBHeEBqAD_BFiDLgGFeE`Ame@zAku@\\{Pt@m_@VoMHeEXaOHiETaLPuFZmHj@aKTqDv@uJRuBd@}EbAqI`Es]vAyLfCuWv@aJ\\}DxA{WVgHb@wQJwHFwPC_FAuDKoIMuI{@{XIeCgAaZm@cQGoAOaE[kIw@iSa@}NeAsWa@iNYaNIcGGyG?[?q@Ak@AoEAiHDyHFkEJeIFaDTuJPuHj@yUvA_n@jAgf@HwCnCylAhAoc@b@iTp@kY\\oNhAqd@NwIRqND{PG{IIcKu@i[oBam@mGioBmAw^eAq[cGokBOoEiAyY_@mHeA{Py@mJa@aEg@iEg@gEm@wEcBuK{@aFw@qEY}@uBeKsA{FqBeIyBaI{D_MmD{J}C{HuAeDgCsFaCaF{HuNoIsMaCeD}EiGuAcBcGyGe@e@}EwEg@k@mC{CoAwAaGoG[]OMcJeGqBmA_C}AwEcEgGgF}GuGaFqFuDiE}AoBuC{DsC}De@s@a@o@iAgBgAeBiAqBaAcBu@uAeAmBeAuBsAoCuAwCwAaDeCcGsAmDuAuD}CeJiCyIqBwHaBwG_@{AcAmES{@YoA{A_HyA{Gg@}BqAcGiAmF_DiNoCyKoBcHwGiSsAwDA[gC{GkCsGyCkHq@kBi@gBo@kCe@gCsAsHm@qC]iAK_@Ys@Qc@S_@Sa@m@}@_@i@i@o@q@m@w@k@e@Yy@_@}@Wo@Mi@GyBG_C@mMNiGHo@EyDNsDTcDh@cGpBuG~CqO~Ku@h@mXbReE|B_DpAwGvAw@LI?EAGAIGKQMIKGGIIKGMe@uAk@{BoGwVnGvVj@zBP|ABV@L@LAXAVBXHR@\\Ah@Ud@e@v@}ArB_C|BqLpIyDfCkFzCo@Zg@T_@PcAXkAXsBZuANgBNe@D_@J]PyBlAMBI?QEQOUCSHQRK^C`@BTDRFNJLJFLPJPFVTxB\\nDRfC@jA?nAM|DWzG_@pLBt@IbCCnC?hB@dBBjBJlCFrAJzA`@bFVjDzAxSTfEFzBBpC?tCK~EOvCUdD]dDk@zD_@tB{@|DeBxFuBlFkA~BuAzBiDhEqDbDkCbBeClAkErAwFp@gCBsCGc[kC{MmAaNiAy@GeHm@aDWsBO{AQcAQ{@So@OoAc@aA_@oDoBiBaBsD}DeBkCaBuCc@aAcCyFw@yA{A}BsAuAiCeByA_A_HyDu@c@SQOMMQEUEIMOSEKBMJSI[Mo@[u@]gAa@kAa@cAYoB[{CWcDQiBIuDSmHg@yCUiE[kFa@qHo@o@Ig@IkAa@{@m@sCyCeBiBW[OSEKCMAYCIAIQ[OISAQHKLEJCNGHGHKJMFWHSLYVUZe@`@WT]VwAv@uQjK}BrA}At@{@Xs@R}@N}@Ls@F}@Dm@?g@?a@C_@C}@IiASeAWoAc@gAi@kAq@_Ao@w@s@y@}@e@g@u@cAa@m@m@aAu@sAYe@W[WYSSUQWQ]OYIy@Qc@Ii@KUIUK[]Y_@OYMYYaAESAO@_@BEBIBM?K@QAMCSCKEIGKGGIEGAKAOBYCOG]U}@q@k@_@UU[]SYWe@a@aAWq@U{@YsAq@_Dc@gBWw@_@aAk@oAs@mAy@kAo@q@i@e@g@a@gAm@s@[qAa@eAQgAMuAEeA@aABeAHqAPcCd@g@LeAPa@Ba@@UYWGWDONIRCBE^?TKNMTURe@Xu@t@y@r@i@h@o@r@{@hAeAfBm@hAm@rAs@nBc@tAe@dBMf@o@nCq@nCaAbEwBlJeCfK_BzG{AtGg@rB[nAUr@Y~@Yx@[t@e@dAi@fAw@rAa@j@i@t@i@n@q@r@{C`DwB|BiGtGy@z@yD~Dm@p@e@f@y@`Aq@x@cC~CQRg@l@oEtFuDxEoEvF_DzDsBjCiBzByCxDqDjEUXQTORmClDeCdDmKxM}BvCkFzGkDnEmCfDoExFcApAo@l@g@d@q@j@]X[P]N[LODOBMGO?MDMJIPEXARBXGb@IZSd@qAlC}@|AORo@z@cAlAgB~Bu@jA]f@}CfFsLjSs@lAmF~IwFpJaBnC}GnLyFrJ_@l@g@x@]n@WVQLM?GBIFCJCJ?H@JQf@Qb@o@dAgAdBaCdEaBpCu@nAqBlDaBlCe@t@SXQXqArBu@v@k@h@uAjAeEpDcDvC_DfC{EhEaFjEaJ~H{IvHmJhI}EdEkThRuD`D[XkIjHk^l[kOrMgIpHsL~J_DnC_GdFkBbBoBbB_GhF_CrBsJpIkJhIc[jXeUbSkDzCoC~BcBnAYFQMS@MHINET?VDVkPnNaAx@qHrGsAhAqE|Dc@`@MPi@jAmDjIkBlEwAfDYf@g@z@G?G@CBCDCF?F?H@FYj@[dA}@vBGPWh@[v@MX]t@Wn@EHk@nA_AvBYp@{@nBc@fA_AxBg@~Ac@bAUh@a@h@O?KDIJEFAHADAJ@NCNCNO\\g@x@mAnBY^g@h@y@~@iAdAyCfC}@bAgApAaA~AiDhF_I~K{FfI_ArAaMhQoDfFsFzHILsC`EsNvS_FbHo@|@mObToAhBu@|@Uf@Sj@e@vAINMHGASDQPMb@?d@FZJRNJHNJ\\R~@^|A\\hA^bAdJxRvEzJn@vAh@xAp@`Cd@xBb@`CR~APjBLjBH|BDnC?lCMvCK`BMxASjBcAhIoEv^cAxIa@jDMt@G^I`@CNGXGXIXERELGLMFGHGLONKDMBO@SAUCWGYCeBy@qEsBc@M_@C_@D{Er@oDj@{AVkMnBeEl@aAPm@DSDKAICEGOMMWQa@Qi@Ye@uB{EsCuGWi@m@q@WOcEcBi@Uy@]}@k@iAu@m@_@YIMCgCkA_@MOCSC[AqDEe@CU?OCMG??UKa@[MGMEMAQB??UDQJ_Aj@]RaB`@WPOROTGTG\\EXCb@GjAEd@CRGRKXIRS`@Q^Wb@QXILIHIHIDMHUJ[Hw@PUDsAX[FyEfA[Jk@X_@Vg@b@c@b@qA`B{AlBcBvBwE~FYXUVe@`@MHEBiAf@}At@uAr@g@Vi@`@[Zk@v@{E`HoEnGs@fAGNYj@}GvOuA|Ce@`Ae@v@}BfDi@t@cH`KeCnDcCnDmLzPkDdFcCnDe@l@WZSPUNk@\\wExBu@\\aK~EyKhF}WjMmLrFsExBUJWL{G`DwC|AQLyBdBuAhAqEvDcCnByGnFwAlAo@l@aApAuHrLsElHeFbI{DbGsApBaBvBiB|Bu@n@m@h@k@r@}A|Am@f@k@d@{@f@i@Xe@Rs@X]HmBb@kANs@DaIPyJT_CFcCD}ACeBD_ABe@A]IGOEGGEMCK@EBKLELAFMDKD_@@q@B{@BiALc@Jk@Hq@LcAVw@ZkAh@m@\\i@\\m@d@i@b@s@n@qIfJi@j@gG`HqJzJiAnAaJ`KY\\QPaCpC}LlNkAvAwB`CORa@d@{BjCq@d@OHMBOBS?I?IGMAUDOLQVITGZA^@VBNFPHLHDNXFPBPDR@Z@XPbFBn@Bp@Bn@`@~HRzDh@vJZbGJxBVvEHxBFpAHpABRXpDHvAB`@@^?^?PAPAPCJEJGLCP?PBNBJDHLHD@JAHEHIDK@K@M?MCKCKEIGGICG?I@GDEFIDIBI@I?KCQCIAO?O@MDOFONIJINIPIZARAb@A^@Z?R@V?H?\\C\\G\\IVKTOPONQHOHqAp@y@b@KFi@\\a@Za@Za@\\_@^]b@]b@]d@Yh@{FvJuD|GaAdBmCxEKNCHmEvHqA|BuCbFOXQVsF~I}ArCwEhIqDnGgArBsBlDaB`D}@pBq@bB{@pCuApHWnCOxCGbCDjCBp@JvB\\zF~@|FlBhJ`C~KBLxC~NtGb[hCnJ`@vAhGtSb@rAxCxJh@fB|CfJ\\lA^vAJv@Pz@VdBJdA?TGTAV@VWt@gBrC}@pA[d@w@nAmA|AiJlM}L|PwBxCoEnGoEjG_LvOuBtCqAtA{@~@yAnA{@n@y@j@gCvAs@b@k@b@}@t@gA|@eAhAeDfDaB~A_BzAcB~B_B`CqAvBsAtCy@zB{@fCgA|D_AjEsHv]yDnP{AtGwAvGmDtPuAhHuAvJo@xEK~ASzBIzAYpCEp@KfBIzBEfCC|A@bCBzADpAL~BXfDPrCPbCNbDDxCIbFGvB[jDk@bEaAzEy@pCsHrU{A|F}@rEg@vEYbFAtDB|EJtCZlD\\nCBRFf@~@dIJ`BHhBC|Ea@nJo@pKATURQRIXCX?f@D^FPRZXPLBRFT?TCB?b@Md@i@TIRChCb@`Dj@pBf@zLhCtGzA|FvAjX`Gp@PhGtApAZh@LjCj@f@LjAZtKbC~TdFlE`AdI~A\\HVLT\\DDFb@LTPLL@TCRSb@MVA~@RhDt@xHdBbJrBvIrB`IdBbE~@dDv@lEbAxA`@`GrAnDt@`@NPP`@d@DDHTNPXL`@TLXHTBV?REXM|@eE`Uk@zC_@vBkCnN_DzPwAlHwBnKyApHMf@GRINEHIFEBEBCBCDCFCDAFAF?F?F?F?F@F@F@V?NAPANCRGZAPIr@i@~Dm@lEM~@Iv@Cr@AbA@v@Bp@?DFv@TbCTtCDnBB|B@zAClCCx@G`AIp@Mp@eAfEW|@iAxEMp@Eb@?^@b@@b@Hp@Jn@Rp@Pd@Xt@fAdCR`@HVBJDJDTBTD`@?`@?\\A\\Gb@Ml@u@xCo@zBQd@cB|EyAhEITg@nAs@jCm@xBEJETG`@E^C^Iv@ObAGXKh@GR_@jAIRSl@GPWr@CHMXINEHSNq@p@CCC?C@Wc@}AsAIGOKOEHj@DVJ`AFv@B\\H|AAh@D`BAPAPATEDADKDMBe@MSESEOAQ?Q?g@HSBOFaDjBg@V_@P}@VgBh@k@TWHqBr@WHWDYBY?iA@c@@wALSBSDw@P{A`@k@NWFa@Fe@FsCNEy@C]GuAAq@AU?WwANSCgAqBuAqCmBqDM]L\\lBpDtApCfApBRBvAO?V@T@p@FtAB\\Dx@rCOd@G`@GVGj@OzAa@v@QRERCvAMb@AhAAX?XCVEVIpBs@VIj@UfBi@|@W^Qf@W`DkBNGRCf@IP?P?N@RDRDd@LHFFBHF@BDT@L@LAL@F?FDRFVNl@Rp@Rr@\\`BNp@\\~AJd@p@zCLh@J^Jb@b@bBjAfE\\lA^lA`@fA\\z@b@|@b@|@^n@d@p@f@n@d@j@\\^`A~@t@v@fAfAhAjAd@j@f@l@b@l@b@p@Zf@d@z@f@|@`@z@h@pA\\z@\\`AX~@Tx@VbAT|@P|@Jb@~DxTxB|LX`BTvAPnALlALrANfBLlBHlBBvAFbEFvBDbBHxAFbAHpA@TFn@LlAJ|@NlAL|@PdA\\rBvAfIXbBNdAJdAJbANfBVbDN`BJpADn@pArQNjBDv@@b@@d@@n@?vB?t@@~A@l@@^Df@Fn@BT@V?H?J@LBHDHFDDBL@JCHKDKBM?K?K?Y?S@SAWEUK[Ok@Sy@Ou@COAO?QBOFKFGNKBATMNILGLGDE@?r@i@tBsAn@]HGLIDAvBuARKj@_@|@k@DCfLeHhJ{FvGcETMpD{BNKXS^]PWr@iAbDmHpAkCXi@Zc@RYT]RQ~GeGxKqJ~FgF~HyGvAkAf@Y^GJ@\\BtCbBjFnDdFrBjBn@lCbAtEx@PJJHFFFH?FBHHPJHJBLA@?JGFKDM@IJEZCF?\\CzCLfHPfFPpDLz@JFBFB@DDDFBF?FE@EHCHALAXCpCJpBFpCHpCHbADtHTfENLDFFDDFTHJFBH@NEHKDKFGFERErENrFNjAFrCRHFHDDBFB@LBHBHFDFDF@FADCDEDGFEH?F?VH`@LPHXJN@PANXNJHBdBr@dBz@|@d@vAv@lCzApBhA`B~@zAx@dIrEj@\\`ExBtHnEnFxCnJnFf@XPJVPzB`CxAxAjIpIb@d@RRJFLHPFvCv@fD|@hBd@vA`@tA^LBj@NVFbAVpBh@v@RbGzAbCn@F@v@TnEjAr@`@LLPVFNJHL@LCFGNGLA^Bp@FnCl@`ATfLdDdQfEv@PpCfAALBNJDHIB?F?DCBITBN?bE@lAPhCKdAJnDt@x@P|ACbAh@f@VDFXj@j@~APn@RbAEBEFAH@H@HDFD@D?BADE@E~@n@j@Xl@Ld@Cp@BtCYj@ERCrEa@\\CN@P@RBhGhBxDb@dDnCpEjBtBdA~@^?JBDFJFBHCDEBGd@Jf@@b@Mv@Ol@Oh@Gp@B~@JfATzEnAd@TTAVGNEPKPOPSPYVo@d@oAXo@`@q@RYTUXUPKZQf@OTEZEbACrBKZKBHBFFDF?DAFCBIBIRLVP^L`@NjC~@dE|ARDP@ZDPEPGP@HBLRJFJBL?RMTATFnCrBf@\\h@Zh@Vz@`@rC`ArAb@fA`@zDrApLhEhC~@VHdHlCfBt@hAj@hDhAPJNPPXHPJFF@JCHGFMBM@QCSEO?g@@GFq@ZeBlFyO?A|@iCvAiEFOTq@f@wADMZu@V_@j@y@dA}A^k@BET_@DIPa@Xw@p@oAr@oAh@c@HIb@e@rByFJS\\Gd@KrAa@P?jDN^CTIl@a@XY^c@Va@Va@b@iA\\m@zHwN~@wAh@q@h@_@f@Wh@Q`@CtEsA^Q^[TUdAoAn@q@`@SZOnGkCz@a@XOd@a@f@c@lAiArAiA^Q\\MtAYnAKbAIv@Mh@Qj@a@v@{@xAyBh@{@bGeL`BoCh@g@b@Wb@Uz@Mf@?VDNNLPHXJPLHLDH?JAPILQJ[BS?YG_@M[QOO]K_@C_AJiAFq@@gBI@]Tm@bAQFOQoAmCw@qAwAsAKWYmCO]c@w@Q_@[u@[g@o@i@_GUsBKS@[Qa@WQOOa@k@sBEOa@eAGU[q@oBsDq@qAp@pAnBrDZp@FT`@dADNj@rBN`@PN`@VZPRArBJ~FTn@h@Zf@Zt@P^b@v@N\\XlCJVvArAv@pAnAlCNPPGl@cA\\UHAAfBGp@KhAGf@Cr@Kj@KLCHIZATBb@HXJPLHLDH?JAPILQJ[BSN_@PSXG\\KnHSX?b@Bf@VT\\Nh@@t@Gf@U`@[Re@A[OWc@Mg@CiAKy@AsE@e@FwE@wARuH`@gLl@sLd@gItAiQp@qH~@mIReB\\gCf@gDBSd@mCtAmHnBaIRg@bAcDjBkGvBkGtBeFvCmGvBeEzBwDjByCzCmE~RkWxNyRdHsKdGiKzFyK|CsGvCwGr@}Av@yBtBkHfAcFl@iDf@qC`@}Cz@yHf@oEVoFPcGBcKGqEIeDQgD]uFi@kGqCeU{@mHqAmNa@{HIeCKsIFeJTiH`@iHT{Ch@kDn@yE`AqFbBkHhBcGRi@zA{EtDcI~AqCdBeCvBuChDsDrBsA~A{A`FkErEaEtDgDlE{D`EoDlBsBhCcDHId@s@fA_BxByD|AaDfAgCx@uBjAeDl@kB^mAPu@b@iBl@iCzAeI~@gFtBuMhC}O~@gGx@gFhAmH^oCXuCX_ERgFJ}FAkFEkDMcDm@yJmAgQqAkQc@}Gc@iH_AsKUeEG}F?}HFkDBw@JkADYVaBh@qB`AeCrAuCb@{@d@e@ZMRIb@G\\BjANpKdEfFvBlPxG`DpAvClAnBv@hAb@nBz@hAb@nBx@|Al@fBp@bGrBh@?xN|FrP|GxAl@h@TvAj@PH`GfC`CnAbB`AfB`ArBtAbDzBnCzBjCzBpCjC~CfDxFnGhXjZfCvCbb@|e@Zp@dArAz@jAv@zATj@`@`ARp@t@|Bp@~BXz@Vt@Xr@^x@l@bAf@r@b@h@~@`A\\`@`BzAjBdBdBbB`BtBdCnDdFtIlDtFlJhPZXrKrQhBhD~JxPpHzMxAjCPh@pGbLjBbDnAhCjArC~BpGTb@b@hAZv@`@|@\\r@^t@`@n@`@n@b@j@b@l@f@f@d@f@f@d@h@`@h@`@r@b@z@`@j@Vl@Tn@Nj@L`Cf@nD`@nBThHl@tCNxHPhCAl@D~GWnGYhFi@lCa@|EaA~EoA|GqBpDmAnHgCxUoI`C{@bA[lJaDbHgB~FmAxFq@j@IpBUtJu@~DY^AxHi@|CStGc@xEItDFlAFvALzBTlAPpCj@rDfAbPhF~C`AhFrAtB`@hATbF~@`Cd@|Cp@h@Lh@NfAZfA\\vCjAnClAhDjBjBnAp@d@fE|C~GdFl@`@vA|@bBn@|@XzAd@zNzDrBp@|@^rCrArCjAvAh@r@X|Cn@fDTfABhAE`AIvAYvBu@x@[rC_BnCkBpGoEbB_AvAq@lAc@|Aa@xEw@bC]vD_APOl@M~@SbAKbAAn@Bp@JbAVbAb@p@b@fA`Aj@p@jAdBhBpCbAlBf@lAb@tAZnAXzAVvBHjAFrB?|CEhCGlDG~BMtFO`J?l@BvBFdBPlBVxAJh@d@nBzAxE~@jCX|@vCxIdO`d@bFvOVv@hCrH~DdKpB|ElB`FpEtL~BpGdDvIrAfDzBzEbEpI|CjFxCtDjCzCtAfBhArBl@pAh@xAt@xCf@hCnB`Mv@xE`@pCHt@PvAF`@FdABvBGtCKnBmBnRAj@OhBAT?X?d@Dd@Hj@H^JVPZTVTPVJNFRDXAXETGVM`Ak@zAgAx@S`AOd@W`J_ArD]`DW~D_@tC@xHb@|H`@z@?n@ATCbC]RG|@WDARG~As@fB}@pBaA|A_@zAAjADvGbAtBT`Fn@dEf@`LtAfANtBPjGPhEPhBPZBf@Hh@HdGdAxHlBrCt@p@Vp@`@RNzA|An@|@b@z@f@jAzDtN\\jAh@~Az@pBzAvCbFzKzCpFjBxCvAfCj@xAp@xBj@hDdDdUT`DZnFN`CL|ALfARzARlA\\pA^zA?B\\zAxClM`@tADRBJH\\Lf@Lh@Pv@Rp@X`A`@lAl@~ATl@n@tAnLnUhBnD\\r@Vh@bAnCzAlGfGnXzDvPlFzUhAfDx@bC^`BXzALp@Jn@Jt@Db@@j@Bn@@|@ChAInB_@jGKjEGhCCrBDfAH|@P^`@\\x@\\b@Rl@XdChBtDhCn@n@r@t@p@`An@lApAxCp@vA~AxDr@zAvA`CtAfBfAz@pC~Br@t@lKvKtHvGxArAn@j@v@j@dAn@fA^xFlA~Bd@nAJl@@vBK`BK|AK~He@B?h@CfEB~@EjAQbCu@r@QhA[`AQb@A`@Cb@AhA?n@@bAJlARj@PhFp@jC\\fAH`A?dAG~@I~@M|@Ur@Sp@UdBo@pE_BrB{@`C}@`A[zAa@zA]`AOz@MjAMx@GvAIfACtB@xABfFZfF^lCNh@BbO~@l@D|Hb@RNpCPnAHfCPx@N|@P~@Tb@P\\LbAd@nAr@nAv@hAt@h@ZjBdAr@Xr@Vv@Rx@NzAPjAFhANz@NfAX|@\\fBz@xDlBp@d@r@d@r@j@l@h@v@t@hApAjC~CjBvBhBrBl@p@xNbPv@v@z@v@FPvHvGvBnB`BzAd@\\d@l@`JxHhHpG`HvGxGpG~A|ArCpCdHdHnBtBbDvDfDnDlBtBfDzDdDpD`CxBdChBtBjAvAp@dBn@vC~@p@Jh@XdAj@lBpAbB`B`DxE\\j@pTt_@zElIdGpKF`@DJBDr@vA`CpGLRpAvD^~@Tj@Zn@Xb@X`@PPt@h@hA^NDD@LFdFb@tGb@rDXbCRpGh@N@r@FL@V@F?Z@lBLpFd@tDRpBPf@BH?HCHGNOJOPk@Nc@Tg@FOBIv@}AdDiGJSJSVs@XkAT_BPoAf@sEFsANqBPeAJm@TsAJqAn@oFd@oEFw@Cm@@UBYRSL]F]@]Ci@GYKWWWOGQCOYGS[uBUeBu@}Fa@yCKmAA}@BiADmA?y@AkAGuAQqAQaA[eAmDcKUw@Kg@Kg@M}@G{@CoAAyACcNCaYCkCG_CA}@@}@BWB]F[F[JWLKDKDQ?MAOEOEKCUA]?k@@_A@{@EwAEkAGcB?wBDaBJcBPqBV_Cj@cFNwAXwC\\iDZ}D@EJsCB}@BkB@cBAkB?s@Au@EyAKwCK{CKmCOqEAm@?g@?_@HcC"
    }
  ]
}
```

## Using a custom matrix

The following input makes use of the option to provide a custom matrix:

```javascript
{
  "vehicles": [
    {
      "id":0,
      "start_index":0,
      "end_index":3
    }
  ],
  "jobs": [
    {
      "id":1414,
      "location_index":1
    },
    {
      "id":1515,
      "location_index":2
    }
  ],
  "matrix": [
    [0,2104,197,1299],
    [2103,0,2255,3152],
    [197,2256,0,1102],
    [1299,3153,1102,0]
  ]
}
```

producing a solution that looks like:

```javascript
{
  "code": 0,
  "summary": {
    "cost": 5461,
    "unassigned": 0,
    "service": 0,
    "duration": 5461,
    "waiting_time": 0
  },
  "unassigned": [],
  "routes": [
    {
      "vehicle": 0,
      "cost": 5461,
      "service": 0,
      "duration": 5461,
      "waiting_time": 0,
      "steps": [
        {
          "type": "start",
          "arrival": 0,
          "duration": 0
        },
        {
          "type": "job",
          "job": 1414,
          "service": 0,
          "waiting_time": 0,
          "arrival": 2104,
          "duration": 2104
        },
        {
          "type": "job",
          "job": 1515,
          "service": 0,
          "waiting_time": 0,
          "arrival": 4359,
          "duration": 4359
        },
        {
          "type": "end",
          "arrival": 5461,
          "duration": 5461
        }
      ]
    }
  ]
}
```
