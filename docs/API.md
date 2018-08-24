<!-- This file is part of VROOM. -->

<!-- Copyright (c) 2015-2018, Julien Coupey. -->
<!-- All rights reserved (see LICENSE). -->

This file describes the `vroom` API.

Contents:
- [Input format](#input)
- [Output format](#output)
- [Examples](#examples)

**Note**:
- the expected order for all coordinates arrays is [lon,lat]
- all timings are in seconds
- all distances are in meters

# Input

The problem description is read from standard input or from a file
(using `-i`) and should be valid `json` formatted as follow.

| Key         | Description |
|-----------|-----------|
| [`jobs`](#jobs) |  array of `job` objects describing the places to visit |
| [`vehicles`](#vehicles) |  array of `vehicle` objects describing the available vehicles |
| [[`matrix`](#matrix)] | optional two-dimensional array describing a custom matrix |

**Warning**: only problems with one vehicle are supported in v1.1.0 so
at the moment, `vehicles` should have length 1.

## Jobs

A `job` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `id` | an integer used as unique identifier |
| [`location`] | coordinates array |
| [`location_index`] | index of relevant row and column in custom matrix |
| [`service`] | job service time (defaults to 0) |
| [`amount`] | an array of integers describing multidimensional quantities |
| [`skills`] | an array of integers defining mandatory skills for this job |

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
| [`start`] | coordinates array |
| [`start_index`] | index of relevant row and column in custom matrix |
| [`end`] | coordinates array |
| [`end_index`] | index of relevant row and column in custom matrix |
| [`capacity`] | an array of integers describing multidimensional quantities |
| [`skills`] | an array of integers defining skills for this vehicle |

### Notes on `vehicle` locations

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

### Notes on capacity restrictions

Use `capacity` for vehicles and `amount` for jobs to describe a
problem with capacity restrictions. Those arrays can be used to model
custom restrictions for several metrics at once, e.g. number of items,
weight, volume etc. A vehicle is only allowed to serve a set of jobs
if the `amount` component sums are lower than the matching value in
`capacity` for each metric.

### Notes on skills

Use `skills` to describe a problem where not all jobs can be served by
all vehicles. Job skills are mandatory, i.e. a job can only be served
by a vehicle that has **all** its required skills. In other words:
job `j` is eligible to vehicle `v` iff `j.skills` is included in
`v.skills`.

In order to ease modeling problems with no skills required, it is
assumed that there is no restriction at all if no `skills` keys are
provided.

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
| `code` | return code, `0` if no error was raised |
| `error` | error message (present iff `code` is different from `0`) |
| [`summary`](#summary) | object summarizing solution indicators |
| `unassigned` | array of objects describing unassigned jobs with their `id` and `location` (if provided) |
| [`routes`](#routes) | array of `route` objects |

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

Using the following input describes a (very) small TSP where matrix
computing rely on OSRM:

```javascript
{
  "vehicles": [
    {
      "id": 0,
      "start": [2.3526, 48.8604],
      "end": [2.3526, 48.8604]
    }
  ],
  "jobs": [
    {
      "id": 0,
      "location": [2.3691, 48.8532],
      "service": 300
    },
    {
      "id": 1,
      "location": [2.2911, 48.8566],
      "service": 300
    }
  ]
}
```

producing a solution that looks like:

```javascript
{
  "code": 0,
  "summary": {
    "cost": 3081,
    "unassigned": 0,
    "duration": 3081,
    "distance": 14422,
    "service": 600
  },
  "unassigned": [],
  "routes": [
    {
      "vehicle": 0,
      "cost": 3081,
      "duration": 3081,
      "distance": 14422,
      "service": 600,
      "steps": [
        {
          "distance": 0,
          "duration": 0,
          "arrival": 0,
          "location": [2.3526, 48.8604],
          "type": "start"
        },
        {
          "distance": 5484,
          "duration": 1108,
          "arrival": 1108,
          "service": 300,
          "job": 1,
          "location": [2.2911, 48.8566],
          "type": "job"
        },
        {
          "distance": 12518,
          "duration": 2417,
          "arrival": 2717,
          "service": 300,
          "job": 0,
          "location": [2.3691, 48.8532],
          "type": "job"
        },
        {
          "distance": 14422,
          "duration": 3080,
          "arrival": 3680,
          "location": [2.3526,48.8604],
          "type": "end"
        }
      ],
      "geometry": "o`fiHaqjMBSPHh@T^PHDTLLHLFJFLFlBz@r@TXN\\NJDHDlCjAJDHDCPCNCNTvBLrAHx@@L@HFp@Oz@~Ap@BBRHPHtDhBHDETKj@Mp@Ov@m@bDI^CHERi@tBGROn@WbBc@`EIfA@TAVAZg@vHGn@p@j@NJRRfB|A@BHLFH}@`BWd@yB|E{@hBEJQpASvAQpASxAAP?DJtC@\\Ab@O~@aArGG\\UhAWlA_@fBOn@_B~GGRCN]|AMf@Kd@gA|EMp@S|@Mp@aBdIADI\\GZEPMp@k@rCGXKf@yEvUQ|@I`@Ml@GXI^?BOr@Mr@CLYtAAT?XEh@Aj@@hNBx@@nA?JBdBBt@@h@?t@?TAFAr@@jDFr@D^LfAJ~@Dh@@`@@f@@J?p@BtCJnNBxD@h@?V@f@DtK?J?bA?F?dA?NGb@?F?^B`AFfA?HDhBFrALhBV~BD\\L~@X|A`@nB~@|C`AfCjAjBLVVNh@z@zE|HNV\\h@PXhDxFl@lA~@fAzAmC{AlCIm@{@_BcEyGQW]i@MTM\\_@~@aErHCBADEHy@gAEGIKe@o@}FcIo@}@g@_BcA_F_@iBAGGWKe@ESAEKe@eBcNMkAe@kGw@{JQiBEc@Ca@Ai@?kACsDCaFKgAAaBKwOCkE?g@C_EAg@CuD?g@CiDIgMC_CAwBBsANaAXkA^gAn@sCXsAJi@~@yDj@_BnEoRBKLe@FWJg@jCoLhBkIVuADSBQl@aC@E@EJ{@R_B@GJq@p@iF@ILaAL_A@EJu@z@aGz@eGBg@BkA?OLcC@OJeB@GLeBLgBBODk@@Kt@yGDa@BU@A?ETuB@KHa@NaABGPcAHe@@It@aEt@mEReADUTqARqA@EJu@FIDM@O?OPaA@GVmAJi@Hc@P}@@EHa@ZqABEFWt@kCVcABGHWPq@BIFY`@wAFUd@iBBGJe@FUPk@DMFOFUZ_APi@hByF@IFSH]@ET}@DSNk@hAcEBId@mBVcABIDSLc@`AsDDM|@iDd@eBHYHWL]v@sAj@_@ZIpAEb@APAH?LAe@eCKe@S}@EQEWy@sDgB{IEOKe@S_AMs@WqAKi@G[Qw@EUWiAKk@COGg@AQAY?OHu@Fy@AQGYIWOQUKSAKBKDEBKLCBKVEXAR?P@RBJ@JDJDJLJTNKpACj@IbAIfACZOxBAHa@rCAJ[bBIj@ERERENu@jDCNGRADe@fBCJGRKb@GRUl@[~@Mb@ENG`@EVCTMz@ETi@fDERQnAYtBEVG\\c@tCCRCPc@vCSrAERCRa@jCOhACLQlAm@dEG\\EKGGOIkAg@IKIOm@i@eAiA{@{@KMIGgAiAIIELw@dDCDi@bCCJcAs@EEEECAIImAoAEHWl@Sv@U`BCVQx@CL`Br@CR"
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
    "unassigned": 0
  },
  "unassigned": [],
  "routes": [
    {
      "vehicle": 0,
      "cost": 5461,
      "steps": [
        {
          "type": "start"
        },
        {
          "service": 0,
          "job": 1414,
          "type": "job"
        },
        {
          "service": 0,
          "job": 1515,
          "type": "job"
        },
        {
          "type": "end"
        }
      ]
    }
  ]
}
```
