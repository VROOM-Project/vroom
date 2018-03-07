<!-- This file is part of VROOM. -->

<!-- Copyright (c) 2015-2018, Julien Coupey. -->
<!-- All rights reserved (see LICENSE). -->

This file describes the API to use with `vroom` command-line as of
version 1.1.0.

Contents:
- [Input format](#input)
- [Output format](#output)
- [Examples](#examples)

**Note**: the expected order for all coordinates arrays is [lon,lat].

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
| [`amount`] | an array representing multidimensional quantities |

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
| [`capacity`] | an array representing multidimensional quantities |

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

## Matrix

A `matrix` object is an array of arrays of unsigned integers
describing the rows of a custom cost matrix as an alternative to the
travel-time matrix computed by OSRM. Therefore, if a custom matrix is
provided, the `location`, `start` and `end` properties become
optional. Instead of the coordinates, row and column indications
provided with the `*_index` keys are used during optimization.

# Output

The computed solution is written as `json` on standard output or a file
(using `-o`), formatted as follow.

| Key         | Description |
| ----------- | ----------- |
| `code` | return code, `0` if no error was raised |
| `error` | error message (present iff `code` is different from `0`) |
| [`summary`](#summary) | object summarizing solution indicators |
| `unassigned` | array containing the ids of unassigned jobs |
| [`routes`](#routes) | array of `route` objects |

## Summary

The `summary` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `cost` | total cost for all routes |
| `unassigned` | number of jobs that could not be served |
| `duration`* | total duration in seconds for all routes |
| `distance`* | total distance in meters for all routes |

*: provided when using the `-g` flag with `OSRM`.

## Routes

A `route` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `vehicle` | id of the vehicle assigned to this route |
| [`steps`](#steps) | array of `step` objects |
| `cost` | cost for this route |
| `geometry`* | polyline encoded route geometry |
| `duration`* | total route duration in seconds |
| `distance`* | total route distance in meters |

*: provided when using the `-g` flag with `OSRM`.

### Steps

A `step` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `type` | a string that is either `start`, `job` or `end` |
| `location` | coordinates array for this step |
| `job` | id of the job performed at this step, provided if `type` value is `job` |

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
      "location": [2.3691, 48.8532]
    },
    {
      "id": 1,
      "location": [2.2911, 48.8566]
    }
  ]
}
```

producing a solution that looks like:

```javascript
{
  "code": 0,
  "summary": {
    "cost": 3679,
    "unassigned": 0,
    "duration": 3679,
    "distance": 14422
  },
  "unassigned": [],
  "routes": [
    {
      "geometry": "o`fiHaqjMBSPHh@T^PHDTLLHLFJFLFlBz@r@VXL\\NJDHDlCjAJDHDCPCNCNTvBLrAHx@BL?HFp@Oz@~Ap@BBRHPHtDhBHDETKj@Mp@Ov@m@bDI^I\\i@tBGROp@W`Bc@`EGfA?TAV?Zi@vHGn@p@j@NJRRfB|A@BHLFH{@fBUd@}BvE{@hBW|ASvAQpASxAAP?DJtC@\\Ab@O~@aArGG\\UhAWlA_@fBOn@_B~GEREP]zAMh@Kb@gA|EMp@S|@Mp@aBdIADI\\GZEPMp@k@rCGXKf@yEvUQ|@I`@Ml@GXI^?BOr@Mr@CN[zA?f@Eh@Aj@@hNBx@D`EBt@@h@?v@?RAFAr@@jDFr@D^LfAJ~@Dh@@`@@f@@J?p@B|CJfNBxD@`AF|L?L?`A?V?dAGb@?F?\\BbAF`A?D?HDhBFrALjBV|BD\\L~@X|A`@nB~@|C`AfCfAhBLT`@n@fAzBtCtEhBvCtCxEt@|@bAfAzAmC{AlCIm@{@_BcEyGW_@S[QNK\\a@~@aErHCBGNy@gAEGMQe@o@}FcIo@}@a@yAeA_F_@iBAGGWKe@ESAEKe@eBcNMkAe@kGw@{JQiBEc@Ca@Ai@?kACsDCaFKgAA_BKyOCkE?g@C_EAg@CuD?g@CiDIgMC_CAwBBsANaAXkA^gAn@sCXqAJk@|@sD\\cAr@yBlD}ONm@FWJe@jCoLjBkITuADQBSn@aC?E@EJ{@T_B?GJq@p@iF@ILaAL_A@EJu@z@aGz@eGBg@BkA?OLcC@OJeB@GLeBLgBBODk@@Kt@{GJw@?GTuB@IHa@PaA@GPcAHc@@Kt@aEt@mEReADUh@cD@ELu@DIDM@O?OPaABGTmAJi@Hc@P}@@EHa@ZqABEFWt@kCXcA@GHWPq@BIFY`@wAFUd@iBNm@FUT}@TYXcAPk@dB_GH[J[@ET}@T_A`AoD@EHWd@mBXgAFYLc@`AsDDM|@iDd@eBDOLa@L]v@sAj@_@ZIpAEb@APAJ@HAe@gCG_@[uAEWy@{DgBsIEOKe@Ow@Q{@WqAKi@YsA]}AKm@COGe@AO?KAKAm@HWFY@Q?SAQGYIWOQSKSAMBKDEBILEBKVEXAR?P@RBJBJBJDJNP@?PHIpAEj@IbAIfAATQ~Be@`D?F[bBKp@CLERERu@fDCNGRCHg@rBENMd@[`A_@jAQd@G`@Il@Mz@q@rEQnA[tBEVG\\c@vCCPCRc@tCStACPET_@fCQjACNQjAu@bFGMGGECuAq@IGIIsByB{@{@KMIGgAiAIIENw@bDCDi@bCCJcAs@OMIImAoAEHWl@Sv@U`BCVQx@CL`Br@CR",
      "steps": [
        {
          "location": [2.3526, 48.8604],
          "type": "start"
        },
        {
          "job": 1,
          "location": [2.2911, 48.8566],
          "type": "job"
        },
        {
          "job": 0,
          "location": [2.3691, 48.8532],
          "type": "job"
        },
        {
          "location": [2.3526, 48.8604],
          "type": "end"
        }
      ],
      "duration": 3679,
      "distance": 14422,
      "cost": 3679,
      "vehicle": 0
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
      "steps": [
        {
          "type": "start"
        },
        {
          "job": 1414,
          "type": "job"
        },
        {
          "job": 1515,
          "type": "job"
        },
        {
          "type": "end"
        }
      ],
      "cost": 5461,
      "vehicle": 0
    }
  ]
}
```
