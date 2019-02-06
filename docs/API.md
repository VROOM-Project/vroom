<!-- This file is part of VROOM. -->

<!-- Copyright (c) 2015-2018, Julien Coupey. -->
<!-- All rights reserved (see LICENSE). -->

This file describes the `vroom` API.

Contents:
- [Input format](#input)
- [Output format](#output)
- [Examples](#examples)

**Note**:
- the expected order for all coordinates arrays is `[lon, lat]`
- all timings are in seconds
- all distances are in meters
- a `time_window` object is a pair of timestamps in the form `[start, end]`

# Input

The problem description is read from standard input or from a file
(using `-i`) and should be valid `json` formatted as follow.

| Key         | Description |
|-----------|-----------|
| [`jobs`](#jobs) |  array of `job` objects describing the places to visit |
| [`vehicles`](#vehicles) |  array of `vehicle` objects describing the available vehicles |
| [[`matrix`](#matrix)] | optional two-dimensional array describing a custom matrix |

## Jobs

A `job` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `id` | an integer used as unique identifier |
| [`location`] | coordinates array |
| [`location_index`] | index of relevant row and column in custom matrix |
| [`service`] | job service duration (defaults to 0) |
| [`amount`] | an array of integers describing multidimensional quantities |
| [`skills`] | an array of integers defining mandatory skills for this job |
| [`time_windows`] | an array of `time_window` objects describing valid slots for job service start |

If a custom matrix is provided:

- `location_index` is mandatory
- `location` is optional but can be set to retrieve coordinates in the
  response

If no custom matrix is provided:

- a `table` query will be sent to OSRM
- `location` is mandatory
- `location_index` is irrelevant

## Vehicles

A `vehicle` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `id` | an integer used as unique identifier |
| [`profile`] | routing profile (defaults to `car`) |
| [`start`] | coordinates array |
| [`start_index`] | index of relevant row and column in custom matrix |
| [`end`] | coordinates array |
| [`end_index`] | index of relevant row and column in custom matrix |
| [`capacity`] | an array of integers describing multidimensional quantities |
| [`skills`] | an array of integers defining skills for this vehicle |
| [`time_window`] | a `time_window` object describing working hours for this vehicle |

## Notes

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

Use `capacity` for vehicles and `amount` for jobs to describe a
problem with capacity restrictions. Those arrays can be used to model
custom restrictions for several metrics at once, e.g. number of items,
weight, volume etc. A vehicle is only allowed to serve a set of jobs
if the `amount` component sums are lower than the matching value in
`capacity` for each metric. When using multiple components for
`amount` and `capacity`, it is recommended to put the most
important/limiting metrics first.

### Skills

Use `skills` to describe a problem where not all jobs can be served by
all vehicles. Job skills are mandatory, i.e. a job can only be served
by a vehicle that has **all** its required skills. In other words:
job `j` is eligible to vehicle `v` iff `j.skills` is included in
`v.skills`.

In order to ease modeling problems with no skills required, it is
assumed that there is no restriction at all if no `skills` keys are
provided.

### Time windows

The absence of a time window in input means no timing constraint
applies. In particular, a vehicle with no `time_window` key will be
able to serve any number of jobs, and a job with no `time_windows` key
might be included at any time in any route, to the extent permitted by
other constraints such as skills, capacity and other vehicles/jobs
time windows.

## Matrix

A `matrix` object is an array of arrays of unsigned integers
describing the rows of a custom travel-time matrix as an alternative
to the travel-time matrix computed by OSRM. Therefore, if a custom
matrix is provided, the `location`, `start` and `end` properties
become optional. Instead of the coordinates, row and column
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
| [`amount`] | total amount for all routes |
| [`distance`]* | total distance for all routes |

*: provided when using the `-g` flag with `OSRM`.

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
| [`amount`] | total amount for jobs in this route |
| [`geometry`]* | polyline encoded route geometry |
| [`distance`]* | total route distance |

*: provided when using the `-g` flag with `OSRM`.

### Steps

A `step` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `type` | a string that is either `start`, `job` or `end` |
| `arrival` | estimated time of arrival at this step |
| `duration` | cumulated travel time upon arrival at this step |
| [`location`] | coordinates array for this step (if provided in input) |
| [`job`] | id of the job performed at this step, only provided if `type` value is `job` |
| [`service`] | service time at this step, only provided if `type` value is `job` |
| [`waiting_time`] | waiting time upon arrival at this step, only provided if `type` value is `job` |
| [`distance`]* | traveled distance upon arrival at this step |

*: provided when using the `-g` flag with `OSRM`.

# Examples

## Using OSRM

Describe a problem with 2 vehicles and 6 jobs using capacity, skills
and time window constraints, where matrix computing rely on OSRM:

```javascript
{
  "vehicles": [
    {
      "id": 1,
      "start": [2.35044, 48.71764],
      "end": [2.35044, 48.71764],
      "capacity": [4],
      "skills": [1, 14],
      "time_window": [28800, 43200]
    },
    {
      "id": 2,
      "start": [2.35044, 48.71764],
      "end": [2.35044, 48.71764],
      "capacity": [4],
      "skills": [2, 14],
      "time_window": [28800, 43200]
    }
  ],
  "jobs": [
    {
      "id": 1,
      "service": 300,
      "amount": [1],
      "location": [1.98935, 48.701],
      "skills": [1],
      "time_windows": [[32400, 36000]]
    },
    {
      "id": 2,
      "service": 300,
      "amount": [1],
      "location": [2.03655, 48.61128],
      "skills": [1]
    },
    {
      "id": 3,
      "service": 300,
      "amount": [1],
      "location": [2.39719, 49.07611],
      "skills": [2]
    },
    {
      "id": 4,
      "service": 300,
      "amount": [1],
      "location": [2.41808, 49.22619],
      "skills": [2]
    },
    {
      "id": 5,
      "service": 300,
      "amount": [1],
      "location": [2.28325, 48.5958],
      "skills": [14]
    },
    {
      "id": 6,
      "service": 300,
      "amount": [1],
      "location": [2.89357, 48.90736],
      "skills": [14]
    }
  ]
}
```

producing a solution that looks like:

```javascript
{
  "code": 0,
  "summary": {
    "cost": 17546,
    "unassigned": 1,
    "amount": [5],
    "service": 1500,
    "duration": 17546,
    "waiting_time": 0,
    "distance": 271905
  },
  "unassigned": [
    {
      "id": 4,
      "location": [2.41808, 49.22619]
    }
  ],
  "routes": [
    {
      "vehicle": 1,
      "cost": 6826,
      "amount": [3],
      "service": 900,
      "duration": 6826,
      "waiting_time": 0,
      "distance": 91070,
      "steps": [
        {
          "type": "start",
          "location": [2.35044, 48.71764],
          "arrival": 29971,
          "duration": 0,
          "distance": 0
        },
        {
          "type": "job",
          "location": [1.98935, 48.701],
          "job": 1,
          "service": 300,
          "waiting_time": 0,
          "arrival": 32400,
          "duration": 2429,
          "distance": 34934
        },
        {
          "type": "job",
          "location": [2.03655, 48.61128],
          "job": 2,
          "service": 300,
          "waiting_time": 0,
          "arrival": 33606,
          "duration": 3335,
          "distance": 49791
        },
        {
          "type": "job",
          "location": [2.28325, 48.5958],
          "job": 5,
          "service": 300,
          "waiting_time": 0,
          "arrival": 35507,
          "duration": 4936,
          "distance": 71311
        },
        {
          "type": "end",
          "location": [2.35044, 48.71764],
          "arrival": 37697,
          "duration": 6826,
          "distance": 91070
        }
      ],
      "geometry": "skihH_wiM??IbC?^?f@@l@NpEJlCJzCJvCDxA@t@?r@@jBAbBCjBC|@KrCAD[|D]hDYvCOvAk@bFW~BQpBKbBE`B?vBFbBDjADvAAz@E|@Gj@G\\IPKJEJCJAL@H@NBHDH@P@`@AZCj@?RA|@@|@F~BBjCB`YBbN@xABnAFz@L|@Jf@Jf@Tv@lDbKZdAP`APpAFtA@jA?x@ElAChA@|@JlA`@xCt@|FTdBF~BA^AXS\\I\\E`@?XBXH\\JTNPXL^?TMRSL]F]@]LYLWRWRW^a@d@e@f@g@`@St@QpAWdBYh@?b@FZHh@X\\^\\`@N`@FRLj@?JIp@W|@CBw@|AOXw@hAa@n@S\\c@z@_@nAIPq@lAUb@M`@EZA`@@^Fj@Nj@Fl@?dAShEK~BOnCK|AOpCqAtQQpCQfC{@xMKvAWrDkAzQIbAGn@KtAg@bHGdA_BfVw@pLg@jG{@pL_ApKiAdKOnAeAdI}@jGkBjK_AdFc@lBq@tCsBvIqBtIiDnMUl@eB~EwBvFwAzDOh@Ij@Gn@Cj@@bAFn@Hh@ZvATv@l@dA^^`@T^Hh@Fn@D|@DhDL|ADhAJ`@D`@HbARdAVzBf@zBr@`D|@|Ab@|SdGxErAj@PhCv@pCx@fF|AdEnAfBh@vBr@jAd@|At@vAx@`Ap@`Ax@v@t@t@t@t@x@fAtA`AtAzAfCdAtBpAnCfB~Fv@xCv@vCtBpIbKpa@vAtF`AtDhA`EjB`HjBfGjAxDh@zAvAbEfBbFh@vA~@fC|@xBnAbDlAzCfB`ErDfIvCxGx@rCrCbK`AlDXlAHf@Dl@@d@Ah@?r@@jA@p@DdAF|@JpARjBXtCr@pHTpBNvAPz@T`Af@fAT\\RX\\XZT\\N\\N~@T`B^vAZ|@^\\P\\Rh@d@d@b@b@h@Zd@`B~BxArB`@p@Pf@Nh@JZXv@DJVj@PX\\n@f@n@RP`Ab@l@`@VXT\\fDbFt@lAp@rAp@fBd@vAj@dBPn@Nl@VjBDr@Dx@@b@A`@?PCJCTC`@APD`@L~BFb@FTFPJPPVTLNPNRHb@l@`KDr@VvEHhBHnBPpGFpB@r@@n@@j@?jB?nCC`BCtA?XGlBGdC?l@AfE@lADjABn@Bx@LxB`@~DLpANtAVxB^rDDdA?dAGtBKtB{@pKIbAEfA?pA@p@F~@Bv@?p@Ch@Gj@I`@GJWLSRQb@Kl@?j@BZH^Xd@XRRNL\\JxAB|@A|AAjAB`AFpA\\zB\\tGB|AHlBH~ALnBTdC\\jD\\nC^nCf@jDJz@x@pGp@jFn@rC^tAHn@AN?NBNDJHHHBHARXnCfGxClGd@~AZtARhARnANjAJnAHnADrABzAAfBEnAKlBQpBW`BWtAa@vAa@nAeAhCyA|CwAhCsAdCQ^MXI\\Kh@Or@GRIHMBMFMJINGPGRANAN@ZBXFVJRJJJHLDL@L?\\R\\`@Vf@\\`AVx@Z~Ah@lDv@~DTpAtBzKhA|Fv@pEXtBdAhIzAjMr@`G|ApHVbCAtAA\\ETGXGRITGLKLOHU@QJU\\If@A`@F`@J\\RRJZBLBXBv@Bz@F~@Dt@B`@D\\Jp@Nz@bDxPjDpQzAdI~AdIrDfRpR~`AwMdEOHMNOb@Mf@SnHG`BCpA[jD]dDQ~AcBxJWvAUtAs@jDs@|@a@b@Yh@]xBs@zEMx@Ix@Gx@I~@OxBUrBMf@w@pDk@jFyAlPcAnFcEtLw@fE]rBUrAGfAAfAFr@F^HRLLPHJJHRFVB`@G?IDEJ?NUGWGKC_AKI@I@OBmB`@_@JM@@\\BHBJRh@Vr@Vp@HTDP@TA\\YbCSfBAX?XQL{AdDAD}@vBe@jA[t@k@zAU~@ETW~ACHIb@YvAOv@ShAM~@Gz@I`Be@`HOzBQzBCn@?j@@l@Hx@@V?TCRKXMRa@b@iAnAuBhC_BrBg@v@[n@Sh@Mf@Kt@YnCI^KZOPYPQFWBSAMGIIMSeFyHIMMGGCE?Q?Eh@yAvKc@nCo@pEKr@u@bF_@hCw@rFEr@E|@Ar@MhTCjCBrAd@`MHtB@|AGpAObDg@lH[jFKn@KHWCuAIo@BU\\q@lAs@bAeAd@kAz@iB\\iAxA{AtCY~A{AnEeBfG_AvDoBpHKb@kAlFu@fCw@`Cm@~AMBKRCN?JINUVk@p@q@x@y@xA]l@]Ze@Xu@Tm@JWHm@d@qB~AgA~@WZIPGLsCeDoA}@gOwJ_Aq@mAeAuB{BqAuBa@}@]eAeGjCg@\\a@n@iGvRI^EHGBQ@QDg@BIPGTEXCtAF`BA\\G`@KVQVwBbDu@hAIXINOLMFMAOCkHmC[UO_@m@cAIb@Ov@_@`BGv@GbAAVE~AYrOAd@Eb@yAzLoCxUu@bGa@dD_@nCUzAUnAsCbOq@nDG\\ETCTTfWDtD@zD@b@@XJj@Jh@D^FHJVNn@Hh@DVL^Vp@Pj@`CtEHNP\\ZZb@R`@JVP\\VLN^_AHELLJXPIHI??[m@OHINID_@~@FFJNHNXbARn@R`@NVJPNN^Xr@f@b@\\Z^T`@n@fAvApCjA~Bf@~@RXPPXRfAr@tBlAjF|CnF~CvCbBbDlBvC|AbCvAxBpA|BlAxP|Hr@\\n@R\\w@h@sBvAoG`AoDLYj@qAlBcD~AcDz@}BPg@\\{@JSLQPMRGb@I`@Gh@Av@?tA@h@?\\At@Gl@Ab@?vACd@Oh@SfCeAjB{@t@Sz@]p@Sl@Cb@BlAb@|@XZ?\\KfAaArAwAzAiCbDmGdAqBv@gApA{@hFcDaBoF_@oBCWIk@^U|DsEZYTMXMZEJ?P@d@JzBv@bNrEz@PD?x@LrNvAXB?F@J@HFHDDB@H?FCHGDO@I?ECQEMGGd@kCj@_DzNcz@@EjFkZtIif@DQ`FuX`@eBd@aBz@qC|Reo@|@wCr@kBl@kAb@o@x@kAbBqBnEwFxRcVlBaCh@o@h@y@d@cAHYRo@V_AVsApAgHZoBPsALsAx@yIHw@L{@Lw@P_AjDuOdBgIXiAVq@f@eAf@aAN_@Ru@Ho@rAj@XLXTb@^PNf@^\\Vh@X\\N^LfAZ~@X`@J~@Zn@\\HDRLx@f@dAf@v@V~@`@hAb@t@Th@RTFRRRRLTJTHPLRHXJ\\TLXNVDRDVFTFVRX\\\\\\`AdA`A~@v@|@dA`Ax@p@x@n@f@ZtAbAfBfAtA`AfBjAtCnBrAz@|AfAlBlAx@d@z@uAT_@PULYAe@?a@@K?QBWBKFUr@}@JSRYFQJc@PgAJ}@Fu@Do@AQCq@?YBg@BWJ_@FONMJGL?XAD?N@PCNCNGPYb@iABGDKTk@t@yBl@oBBIRm@\\gAPo@^qAV{@J_@VaANw@BMJ]@Gx@jAbCjDlAbBn@|@`@l@`@j@Xb@V^Xf@NX\\t@p@|Ap@zAn@zA`@jAZ|@Vx@t@~BRl@L\\`@~@Rb@Xj@RZXh@NTb@n@Zb@d@j@NPXXTVb@`@TPd@\\XTXPXPXN^RVLXJXJ`@N^Jz@VjAb@^Pz@VP?ZJTHPDb@JdAZh@Nl@Ph@Pb@L`@Tn@\\^Tj@`@LJt@x@f@d@Z^VVl@v@PXDSBGDKNe@L_@Ty@|AuFdCqJd@iBtAsF??r@kD|@qDz@{Cx@iC~@iBHQjH}MbByCbB}DtByDp@sAVa@HOFK^e@HKVUfA_AlAsALObBkAXQ|BuAb@Yn@e@LIhA{@fB{ArAgAvAsAxBmBVUVUzM{NROzAiBjCmCbDkDRS`B_BTS|@{@bBcBLKRW|EmEtAgBtAiBtAgBJMt@iAXY`AeAfAaAhAmAHMBDFDD@D?D?BABC@G@ICIAE\\UXM^MRGv@UxBk@LEFCDC@CBE?GHcBTeCHkA@kAC{@E}@q@aEe@gBI_@E]CkACw@IiEAw@B[@ODKf@u@^c@POPGXKtASx@IjDUpAK`@Ib@Mb@Wb@]f@u@P]Ne@dBeGfByFx@sBv@wBp@gCn@gC|A_Gn@aCp@oBdCaG~@_CZ_ATgAlBuI~AwEjBeEx@kCvAmGpBgF~CcI~CiGBEvEaItAaC~AuE^s@`@o@tAcBT_@Pa@z@mCfAeCDM?WGeD?_A@ODc@RFnAVAy@B_ABm@L_DBy@@_AAq@KyAEa@Os@YgBI{@@mABeADe@RaBnAgEv@wDNmAJuAH{AByACkBIcCYgEIyACqA@qA@mFA{@Au@CaA]aFy@iMkAsRq@yI[kESuBQwCQwB[yKu@_WIwOGeK?}@?e@Ag@Ck@E_A@_@JY@YGYCa@Cw@A{A?e@@g@FmA@e@?WA_@Ac@Ea@Gm@Gy@Iq@S{B[eCYkBWqAWiA_@cB]{A]_B_@uBQq@Sg@Ug@k@iA{@kBcCcFk@iAc@cAUi@Qo@Os@Ks@Ky@IoAKkCKaDSqGe@sNa@{K[}Ja@cMEiA]sJC[EkAAu@Au@Aw@B{D?uB?eA?oAEuAEmAGsAIy@OaBQqAUaBSuAoA}HKiABg@Ha@HGHMJ[@a@E[CIGOOOKa@[s@kAkDaAiEYkBUkBSsCKmDGmFAwAE_BIgAWwCOwAQgAUiAk@_Ce@cBs@yBwBoGgB_Gm@_Ci@oCYkBUcBYsCe@cG[mEa@mEc@eEYsB]wBa@qB]yAi@wBu@oCgCuImFkQ]uAk@aCEY?YDU@]E]IWMMMEKAK@OGKSIa@Mq@Uw@Uu@Yk@mCaJgCqIg@cBe@}Aa@cBa@kB_@sB]gCOcBM{AIsBIqBIyDK_GSgJHmBBsAB{@Bk@Fs@Fi@H_@NIJSBUAWESKMGuAKoBKyBIy@My@]}Ai@eC]uAi@{B_AeEg@qDu@kGM{@[qAUq@[m@e@m@OQYS]Oi@O[Gm@AsC@[I_Aa@_@s@Se@E[IYKUGo@?WBc@Fi@n@sJVuDDu@`@eGf@aHT}CFs@Hm@L_AN}@Ps@Nk@Xo@Xe@\\?ZKTYPc@Dg@?m@Ig@F]DQHMLGTCv@CXCJC\\I@C@C?C?w@?W?cA?sB?eA@YBSFUl@yBdAuD?W?OAOEMEMe@k@g@q@MOe@k@EEIOGKCQCSAW?W?_@@YBSDSXiAHa@Lu@BYHy@@_@@g@@UDQFQHMTYvBoCT]bAoAZk@RNNSz@}AJQR[FMFODKBM@M?MAMAQGc@Gi@E_@E[Ok@CQC[AWAOAS?Q?i@@c@AWAa@Cc@Ca@Ga@Q_A?M@O@KDKFGZYRSNQRYRSXm@La@lAkCRc@~@sBRm@WWaD_DM]@YBM?MCOEEAEICc@k@Qc@I]w@sEKy@Ew@GeAA[AWA]W{DCQAk@BqCDqD?k@Ba@FcDAg@@i@BoAFoABeA@GBID?DCBI@IAGCGECG@CO?CMoAGg@Am@EuAAoBCcAKgB]cD]sDQoAY_BYqAEWa@mBQo@Qm@IYG]CM?ECQAU?GC]?[A[@s@As@As@A[Cg@EWEQFEDI@KAK??EIGEG?GDEHALBJBHFDFADPDVBf@@Z@r@@r@Ar@@Z?ZB\\?F@TBP?DBLF\\HXPl@Pn@`@lBDVXpAX~APnA\\rD\\bDJfBBbA@nBDtA@l@Ff@LnA?BBNCBCHAFBLFDCHAFCdAOAGFUZUXMJKHEDG@QFQBMDSCYCO@KDmAb@m@VYNIFIFKLMVMVw@fB_@bAQb@O^AFy@rDg@fBQl@Ob@Ob@Q\\Q`@a@x@OXUZ}CfEGJu@_@eAe@e@]k@e@o@e@w@i@w@a@}@c@w@Ws@Wy@Qw@Qw@K}@I{@Cu@CeA?]AWCc@K[Mm@[y@_@MOYt@SLQHABAB?D@DHVTl@zA`E@ZE^YjAs@~BGVEHYhAwBzGGHEDM@KC}@i@w@m@_As@MIYWy@w@a@Ys@e@q@_@yC{@[K]W]Yy@{@c@c@Q^aBlCKP[f@c@t@g@v@IFEDi@ZOHKNa@z@CJALMbBC^GjAEXMt@GT[l@EKgAiBEGGCw@Dc@B_BgAIGGHk@r@?@CHwExPIZABGRuCdKIVOb@uBiAoDkBgHuDeJyEsAu@iDgBmAo@sAo@{Ao@q@SmCs@aCi@yHcBmJqBmBa@_A[y@[uBeAoDkB{K{F}C_BECe@SKEeA[e@Qe@OkDgA{DmA_Co@wAc@}EwASG{Ac@OEOEaSsFe@MiF{AgGcB{@UqDgAwDgAyDeAuDeAkGkB}DiAqIeCeWcHmJmCSEu@UqJoCoJoCuCw@oCw@EAgF{AoA_@{EsAgGeBqCw@sIcCcGeBoA_@i@OoCs@iRqFsYiIyLkDoDcA}DgA_BWu@Ko@Em@EqAEs@?s@?yNf@kSp@m@BsDNkBFI@[@gCHmADS@mCJoHXuCJiADiA?aACaAG_AIy@Ks@Mq@Oo@Oi@Ou@Wu@Yk@Wg@YqAs@iAu@m@c@gBaBo@o@o@s@m@s@k@q@uL{OwL_PgCiDgB}BwAmB}ByCgAsAi@i@g@e@c@_@m@e@c@YSK_Bs@KGIICKPk@Nc@`@aAv@}AdDiGJSJSVs@XkAT_BPoAf@sEFsANqB\\sBTsAJqAn@oFd@oEFw@Cm@@UBYRSL]F]@]Ci@GYKWWWOGQCOYGS[uBUeBu@}Fa@yCKmAA}@BiADmA?y@AkAGuAQqAQaA[eAmDcKUw@Kg@Kg@M}@G{@CoAAyACcNCaYCkCG_CA}@@}@BWB]F[F[JWLKDKDQ?MAOEOEKCUA]?k@@_A@{@EwAEkAGcB?wBDaBJcBPqBV_Cj@cFNwAXwC\\iDZ}D@EJsCB}@BkB@cBAkB?s@Au@EyAKwCK{CKmCOqEAm@?g@?_@HcC??"
    },
    {
      "vehicle": 2,
      "cost": 10720,
      "amount": [
        2
      ],
      "service": 600,
      "duration": 10720,
      "waiting_time": 0,
      "distance": 180835,
      "steps": [
        {
          "type": "start",
          "location": [2.35044, 48.71764],
          "arrival": 28800,
          "duration": 0,
          "distance": 0
        },
        {
          "type": "job",
          "location": [2.89357, 48.90736],
          "job": 6,
          "service": 300,
          "waiting_time": 0,
          "arrival": 32247,
          "duration": 3447,
          "distance": 62693
        },
        {
          "type": "job",
          "location": [2.39719, 49.07611],
          "job": 3,
          "service": 300,
          "waiting_time": 0,
          "arrival": 35908,
          "duration": 6808,
          "distance": 118600
        },
        {
          "type": "end",
          "location": [2.35044, 48.71764],
          "arrival": 40120,
          "duration": 10720,
          "distance": 180835
        }
      ],
      "geometry": "skihH_wiM??IbC?^?f@@l@NpEJlCJzCJvCDxA@t@?r@@jBAbBCjBC|@KrCAD[|D]hDYvCOvAk@bFW~BQpBKbBE`B?vBFbBDjADvAAz@E|@Gj@G\\IPKJEJCJAL@H@NBHDH@P@`@AZCj@?RA|@@|@F~BBjCB`YBbN@xABnAFz@L|@Jf@Jf@Tv@lDbKZdAP`APpAFtA@jA?x@ElAChA@|@JlA`@xCt@|FTdBF~BA^AXS\\I\\E`@?XBXH\\JTNPXLT`@HTBVFv@e@nEo@nFw@~C[tBcAnKGf@Eh@U`B_@rA_@`AsAhCuAjCo@jAg@fAk@xAUl@UC]CqBMsDYsFi@}AIq@I[Cm@EOCcRwAsBu@qCeA_CoAQKkByAeB_BkDsEsEuG]k@wDuG{CiFiEqH_DoFqCeFqCoFqDqH}@eB[m@wBcEqA_Co@oAs@sAyA_C_B}ByBwB{B}AyEoC_FiCaDmBsAaA_BoAoGqFiM{LgB{A}BoBaAw@cBqAqGaFcH_G{FcFmG{FU_@wEoEs@cAg@_Ao@}AoAgEc@mB_@qBOiASaBQqBImBEsBAoBB{AFkBHoAJ}AAi@jAoOz@wLj@{Gb@oGFs@\\_Fx@oKZoERcEJaDCs@V_N?mL@iC?{CWu]QwUCmD@{A@w@DaA`@oLLoAd@mM?aB@qBKeCUeCMaAQkAYoAa@iAkAsDCI{@mBg@_Ae@w@eAqAs@s@gAy@WMaAe@eA]iAW{@I{BG}@DM@u@N{@Tw@\\mBhAeRtKwR~KaBdA}@\\s@Nw@Bu@@w@E{@Qc@OqAw@g@e@k@u@i@_Aa@}@a@oA[uAQiAK}@G_AC]mCwb@iBkZeAmMk@uEMgAs@gFuAkG{AqIgAyHq@qF[qEOwFCcI?aDA}AMaCAQOqB}@}EiA_EyCsJwA}E_AmEa@wDKmCAoBXuJ`Ak[By@NuEF_D@{CAsBQcNAm@GcEKyIEwDCgCC_BGgDEyAGmAKsAQyAO_A[wAc@_Bg@sAo@sAeA}A_AaAs@o@q@a@_Aa@}@WiAQeAIgA?gALcKzAUDsF|@uKv@kHDqBCe@EqEU{Eq@wB_@cCe@uAYSCuA[qDmAiC_A{FmCqBeAy@m@]]yE{CmByAgA_AgCwB_CsBQOkEkDqC}AiCiAeBo@gBi@wBe@m@IoAMoAMqAGqAGy@EcJQaBCo@CcFK_CGy@CmADs@Aq@?s@@i@Bs@FyALeBJ_AAoAQ}Am@w@k@q@o@y@iAo@gAi@uAe@}A]oBo@wDKk@e@oBa@wAyAeDo@{AsAyC{BgEWOYq@oDuI{CaHuAoCaA}AqAoAyAaAkAm@_BYuDm@aBa@sAk@mA_AiAmAo@mAy@aBe@yAc@iA]qA_@iBYwA]kBg@eCe@mCg@gDq@}Fy@cKqBiVe@aH]{DcAiM{Be[u@kMY{KAgH@qCBoCFeCN_DR{CXqC^}Cp@eEhA{FzA_Ij@aDp@yDx@_Hf@iE^wDXiDLeBNuBp@}Ir@gQv@iWR}FB}@FcDBeDG_EKsCQgCSkBYyB_@qBk@gCo@yBu@sBs@aBcAmBe@u@iAcBkD}EqB{CaA}A_BmCcB{CcBaDy@_BwBkEeAuBkCwGo@cBgAwCc@gAeBkFkBqG{@mDo@cE]cDSmDEoD?s@?u@BoBBqBHeEBqAD_BFiDLgGTeJz@m`@rAku@`Aya@h@_]HeEXaOHiETaLPuFZmHj@aKb@}Fr@yIh@kFhAiK`Es]vAyLfCuWv@aJ\\}DxA{WVgHb@wQJwHBiBByGAmG@}FSkNKoF{@cYIeCgAaZm@cQGoAOaE]kIu@iSc@aNcAoXa@iN[aNGcGGcRCwB@qDDyHDeEFeFHgETuLPuHj@yUvA_n@jAgf@HwCdCshArAug@x@s]~@m_@dAqd@PuIPsNFyPAyIKgK{@c[sBqn@kGcnBmAw^eAq[oFqbBc@mNeAoY[iGmAmRy@iJkAqKsAyKuEmY}@iFsAuEsA{FqBeIyBaI{D_MmD{J}C{HuAeDwF{MiKkRiKwOeQiRaGaGuDgEoAwAaGoG[]OMcJeGkBuA}IyGeGoFaO_OuDiEcBwBcDsE_C}CgAcBiAgBgAeBiAqBaAcBu@uAeAmBeAuBsAoCuAwCwAaDeCcGsAmDuAuD}CeJiCyIqBwHaCsJcAmES{@YoA{A_HyA{Gg@}B{CqN_DiNoCyKoBcHoGmS{AsDA[gC{GkCsGyCkHq@kBi@gBo@kCe@gCsAsHm@qC]iAK_@Ys@Qc@S_@Sa@m@}@_@i@i@o@q@m@w@k@e@Yy@_@}@Wo@Mi@GyBG_C@mMNiGHo@EyDNsDTcDh@cGpBuG~CqO~Ku@h@mXbReJnEwGvAw@LI?EAGAIGKQMIKGGIIKGMe@uAk@{BoGwVnGvVj@zBP|ABV@L@LAXAVBXHR@\\Ah@Ud@e@v@}ArB_C|BqLpIyDfCkFzCo@Zg@T_@PcAXkAXsBZuANgBNe@D_@J]PyBlAMBI?QEQOUCSHQRK^C`@BTDRFNJLJFLPJPFVTxB\\nDRfC@jA?nAM|DWzG_@vOI`FAbCFvDJ|CR`Dr@zJ`BlTTfEFzBBpC?tCK~EOvCUdD]dDk@zD_@tB{@|DeBxFuBlF}C|FmDfEqDbDkCbBeClAkErAwFp@gCBsCGc[kC{MmAaNiAy@GcBMcJw@sBO{AQcAQ{@So@OoAc@aA_@oDoBaDuC{BiCeBkCaBuCc@aAcCyFw@yA{A}BsAuAiCeB_FsCyCeBu@c@SQOMMQEUEIMOSEKBMJSI[Mo@[u@]gAa@kAa@cAYoB[{CWcDQyBO}Cq@m@OmA_@y@Eq@@c@D_@Ba@?IUKOQKQAQFOLKTGZ?\\DZJVFx@OfDc@nIyAvUe@`Iq@vMOrCUpF{@~US|Eg@bNq@|QMhEMrGCtLItPJxF\\nRRbI|@hd@H~KD`Cj@|YXjMF`EDrAd@tS~Apv@J|CRlDr@~FvFdb@bBxLzFzb@xBlPpBlON~@`AfHjGhd@n@hFpFv`@dBnMTxBP`CPzEPpNV`OBzAR`Mf@pXv@zb@x@~b@p@|`@LxHZrMLxHL~Gd@`XH|EZ|PBjAH|D?HFtDBlDGdFKzCKzBOxBSvBi@pFQ|As@tGs@nHw@~HqBxR_@hDOnASvBQlCMpCErBAtC@pBHbCJzBPxBXbCZbC^xBj@bCXbAXbAVt@f@rAZt@dAzBdBdDnBfE~BxFfAvDx@nDt@lE^lCXxCPnCLnEpC`_AzJtjDP`L@`BGpBKt@Ij@Up@g@x@k@d@q@ZsDLkPa@cL[qGImEBuBH}@JuB`@kB^}Bl@e@J]LwCpA_@TqAl@{A|@sBrAoBhBy@t@yAxAgB|By@dAeB`C}@zAs@xAu@xAs@vAu@dBmA`DSl@gB|E_ApCwT`p@cEbNwQzi@oTnp@qFlPeD~J]hAgGtQqArEWv@a@xBUdCGrA?xAHvANvA\\~A^jAfArBdAtAhGvGhBzCjBtEl@bAnDtKbAfDnErN|GpVjHzZxJ`c@Cj@N|@Lz@b@~DLdAd@tAKj@?n@KXaAl@eBv@oAp@kD`BWJ[NiFdCwBdAWLSJ}@b@kElC_@TGMKIMCMBKHGNCRAROPk@b@}@X}@FaAGkEu@wB?}A`@cBv@cB`BwA~BaA|Ce@zBs@|E_A|F{AvJ}A|JqApI{A~JgB~Km@rDI`BAhBF`DTbCf@|CVbAf@jCh@jDt@bEn@zCFPCNKPSHsBLmMv@S@cG\\qAHOEOQGUKQOQUIUASDQLKNIPGRCf@G^IPa@\\c@d@]l@Q\\]`BKd@m@`Cu@xCa@~A_@bAMTc@n@e@l@g@f@c@Vc@Vi@To@T{AZw@Ns@Hy@T_AZc@IAKEIGEGAG@EDEDCFAHa@VqA`@qARiB`@YJcBp@gB~@{@`@iAb@UJULSLURMHWX]b@URIBQFC@IFGJCL?L@LCt@[hAsApDOXm@zAc@x@u@fAg@t@_B~AoA`@o@F_B?i@WGMGGGAI?IDGHER?N@NFLBBHDTp@?B\\xA|@bCFj@KZC\\?^D^M\\a@fAcApBu@tAs@`Ae@^]NYFIIMCK@KDKLGNCJ?LMTSPQL_IbCmFtAqJ`C}Bv@yAp@i@\\i@b@kAdAWHW@EKCCGKOIQAOBEDIDKPGTCX?XBNBNFL?R?PCVIXq@jBsCpKkCrKqCnLsB~Ha@fB[zASdAMn@Uj@OJKPIVCV@Z_@r@GRg@|@{@nAq@bAs@p@y@T{@?w@?}Ak@cOmDwFmByOkI}JsFiEeCaB_AwImF}BiB}@q@s@c@c@Qi@Qo@Mi@Ae@@g@De@HiAVo@La@D]Bc@?k@G]Gm@KSKKIQQ?EAEAEAEAEACAEAEACACAEACACCCAEACCCACCCACCCCCCAACCCCAAAAAA?AAA??AA?AAA?AAA?AAA?AAC?A?AAA?A?AAA?A???A?A?AAA?A?A?A?A???A?C?A?A?A@C?A?C?C@C@C?C@A@E@GBABC@C@A@A@CBCBEDCDCBCDCFCFCFCDCDADADADADAD?DADADAF?FAF?HAF?F?H?F?F?F@D?F@F?F@D@J@HE|@YnCe@~DWnBu@|Es@|De@rBkBlGqCdGmBvD{BtD_AvAc@l@MPiC|CiBfBeEnDqE~D_EnDgErDeFhEqE`EqD`EuBrCeBhCwAhCcBlDyA~CoBnFiBjGaB|HeApFo@~Ea@pDWzC_@hHQjHE`J?bDF|DJ`Ch@vJ`@hFh@pEh@fF~B~Rz@tIRzCLbBLzBLxDHzCB|DA~FKzE_@fJm@bHi@`FaBfJ{@nDwAvFwBpGuEtKaI`P{NrViIbLmM~P{MhQgGlI}C~DgDfFqC~EeAfCqBnFaAbDkA~Eq@tDc@bCcCtMcA|Fi@hDq@pFi@jFg@hF}AnQSlDe@xHm@rLc@zL[fMExCAx@A~FAlAE`@Kh@]b@_@T_@FY?q@Qe@Om@a@a@_@KYE_@Ac@BS?YG_@M[QOO]K_@C_AJiAFq@@gBSFSNm@bAQFOQoAmCw@qAwAsAKWYmCO]c@w@Q_@[u@[g@o@i@_GUsBKS@[Qa@WQOOa@q@cCa@eAGU[q@oBsDq@qAp@pAnBrDZp@FT`@dAp@bCN`@PN`@VZPRArBJ~FTn@h@Zf@Zt@P^b@v@N\\XlCJVvArAv@pAnAlCNPPGl@cARORGAfBGp@KhAGf@Cr@Kj@KLCHIZATBb@HXJPLHLDH?JAPILQJ[BSN_@PSXG\\KnHSX?b@Bf@VT\\Nh@@t@Gf@U`@[Re@A[OWc@Mg@CiAAqHAyE@wARuH`@gLl@sLd@gItAiQp@qH~@{IRwA\\gCf@gDBSDUtBeLnBaIRg@bAcDjBkGvBkGtBeFvCmGvBeEzBwDjByCzCmE~RkWxNyRdHsKdGiKzFyK|CsGvCwGr@}Av@yBtBkHlA}Ej@{Cn@uDj@yDl@}F^_FVoFPcGBcKGqEIeDQgD]uFi@kGqCeU{@mHqAmNa@{HIeCKsIFeJTiH`@iHT{C`@mDp@}EbAuFbBkHhBkGrBsFtA}C~AeD~AqCdBeCvBuChDsDlE}DbFkErE}DzDkDhEsDfEuDlBsBdCwCHId@s@fA_BxByDnB{DbCmGx@yBx@oCd@}Ab@gBb@iBd@yB|ByMtBuMhC}O`@iCzAeJfAcH\\yCXuCX_ERgFJ}FAkFEkDMcDm@yJmAgQo@yJYaEoAwQ_AsKUeEG}F?}HFkDBw@JkA\\{Bh@qB`AeCrAuCb@{@d@e@d@U^Mh@Av@NVJt]vNpJnD`DnA`Bp@tAj@`Bp@xAl@bBt@~Al@fBp@|MhElIdDnP~GxAl@h@TvAj@PH`GfC`CnAbB`AfB`ArBtAbDzBnCzBjCzBpCjC~CfDxFnGhXjZfCvCbb@|e@Zp@dArAz@jAv@zATj@`@`ARp@v@zBp@~BL\\f@|AVj@hA|BpBfCp@h@rEpEdBbB`BtBdCnDdFtIlDtFlJhPZXrKrQhBhD~JxP`KzQz@vANf@`FxIjBbDnAhCjArC~BpGTb@b@hAZv@`@|@\\r@^t@`@n@`@n@b@j@b@l@f@f@d@f@f@d@h@`@h@`@r@b@z@`@j@Vl@Tn@Nj@L`Cf@nD`@nBThHl@zFZrEDhCAbBAhFO~BOxJs@lCa@|EaAlAWnLiDpDmA`KmDfSiH`C{@bA[lJaDbHgB~FmAxFq@j@IpBUtAI~NgAxHi@|CStGc@xEItDFrIv@`JxBbPhF~C`AhFrAtB`@hATvRfEf@JfA\\vChAnClAhDjBjBnAp@d@fE|CvAhAtF|DvA|@bBn@|@XzAd@zNzDrBp@tAl@zBdArCjAvAh@r@X|Cn@dD\\hA?hAI`AIvAYvBu@v@WtCcBnCkBpGoEbB_AvAq@lAc@|Aa@fDk@tDi@nAW`Bk@dAY~@SbAKbAAn@Bp@JbAVbAb@dAt@r@n@j@p@jAdBhBpCbAlBf@lAb@tAZnAXzAVvBHjAFrB?|CGlDEhCG~BWfMCnCAl@BvBFdBPlBVxAJh@d@nBzAxE~@jCX|@vCxIdO`d@bFvOVv@L\\xBtG`EdKpB|ElB`FpEtLjExLxAnDrAfDzBzEbEpI|CjFxCtDjCzCtAfBhArBl@pAh@xAt@xCf@hCnB`Mh@tDv@rFR~@NvBBvBEbCQrCiB|QOd@Iv@K|@C^?^@`@Bb@F`@J^Ph@Zj@nBvCRFN@TENEVKLKh@q@JSt@yAVUXOf@QhAW|B]lB[rDo@lBi@d@CbD]~D_@vCEvHj@|H`@z@?n@ATCbC]RG|@WDARG~As@fB}@pBaA|A_@zAAjADfEp@dEf@`Fn@dEf@`LtAfANtBPjGPhEPhBPZBf@Hh@HdGdAhGtA|Bh@v@T~@b@fAt@xAzA`A~Ap@~AbEzN\\jAh@~Az@pBzAvCbFzK|EnIh@z@vAfCj@xAp@xBj@hDdDdUT`CVnET~DL`BJdARzARlA\\pA^zA?Bl@tChCrK`@tADRBJH\\Lf@Lh@Pv@Rp@X`A`@lAl@~ATl@n@tAdO`Zp@pAVh@bAnCvA|FfFnUb@nBdEpQ~DjQNn@Pp@d@dBd@tAx@bC^`BXzALp@Jn@Jt@Db@@j@Bn@@|@ChAInB_@jGStICrBDfAH|@P^`@\\x@\\b@Rl@XdChBpDjCXTXVr@t@p@`An@lApAxCp@vA~AxDr@zAvA`CtAfBfAz@pC~Br@t@lKvKtHvGxArAn@j@v@j@dAn@fA^xFlA~Bd@lALn@?vBK`BK|AK~He@B?h@CfEB~@EjAQbCq@~Bm@bAQ^E`@Cb@AhA?n@@bAJlARtGbAjC\\fAH`A?dAG~@I~@M|@Ur@Sp@UdBo@pE_BrB{@`C}@`A[zAa@zA]`AOz@MjAMx@GvAIfACtB@xABfFZfF^lCNh@BbO~@h@DbJ`@bCLzEZjAL~@Ph@Lp@PhAZhAn@d@Vh@Zh@^j@`@l@`@x@d@hAn@zAh@hAXjAPxCXfBVbBf@jCpA|DtB~AjAbBjAdClCrKxLpO`QnFzEdIhHhNzLrHtG`@^bGlFzG~F~AtArCfCbPtNbA|@tC~CxGhGzAdBnAlBpB`DhAbB~@|@fA~@~@p@lBdAbDlAl@^h@XdAj@lBpAbB`B`DxE\\j@pTt_@zElI~B~DvCdGr@vAnC~GpA|D^~@Tj@Zn@Xb@X`@PPt@h@dAXbDj@tAJhHd@rDXbCRpGj@P?n@HN@^Bj@?|AJpFd@tDRpBPf@BH?HCHGNOJOPk@Nc@`@aAv@}AdDiGJSJSVs@XkAT_BPoAf@sEFsANqB\\sBTsAJqAn@oFd@oEFw@Cm@@UBYRSL]F]@]Ci@GYKWWWOGQCOYGS[uBUeBu@}Fa@yCKmAA}@BiADmA?y@AkAGuAQqAQaA[eAmDcKUw@Kg@Kg@M}@G{@CoAAyACcNCaYCkCG_CA}@@}@BWB]F[F[JWLKDKDQ?MAOEOEKCUA]?k@@_A@{@EwAEkAGcB?wBDaBJcBPqBV_Cj@cFNwAXwC\\iDZ}D@EJsCB}@BkB@cBAkB?s@Au@EyAKwCK{CKmCOqEAm@?g@?_@HcC??"
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
