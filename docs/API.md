<!-- This file is part of VROOM. -->

<!-- Copyright (c) 2015-2016, Julien Coupey. -->
<!-- All rights reserved (see LICENSE). -->

This file describes the API to use with `vroom` command-line as of
version 1.1.0.

Contents:
- [Input format](#input)
- [Output format](#output)
- [Example](#example)

**Note**: the expected order for all coordinates arrays is [lon,lat].

# Input

The problem description is read from standard input or from a file
(using `-i`) and should be valid `json` formatted as follow.

| Key         | Description |
|-----------|-----------|
| [`jobs`](#jobs) |  array of `job` objects describing the places to visit |
| [`vehicles`](#vehicles) |  array of `vehicle` objects describing the available vehicles |
| [`matrix`](#matrix) | (optional) two-dimensional array describing a custom matrix |

**Warning**: only problems with one vehicle are supported in v1.1.0 so
at the moment, `vehicles` should have length 1.

## Jobs

A `job` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `id` | an integer used as unique identifier |
| `location` | coordinates array |

No two jobs should have the same `id`.

## Vehicles

A `vehicle` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `id` | an integer used as unique identifier |
| `start` | coordinates array |
| `end` | coordinates array |

No two vehicles should have the same `id`.

### Notes on `vehicle` locations:
- key `start` and `end` are optional for a `vehicle`, as long as at
  least one of them is present
- if `end` is omitted, the resulting route will stop at the last
  visited job, whose choice is determined by the optimization process
- if `start` is omitted, the resulting route will start at the first
  visited job, whose choice is determined by the optimization process
- to request a round trip, just specify both `start` and `end` with
  the same coordinates


## Matrix

A `matrix` object contains a custom distance matrix for the TSP/VRP, 
as an alternative to the distance matrix provided by OSRM through 
the locations. Therefore, if a custom matrix is provided, the `location`, 
`start` and `end` properties become optional. Instead of the coordinates, 
an index has to be provided for a row indication on the matrix:

| Key         | Description |
| ----------- | ----------- |
| `job -> index` | an integer used as unique identifier |
| `vehicle -> start_index` | row_location for the starting point |
| `vehicle -> end_index` | row_location for the destination point |

### Notes on `jobs`, while custom matrix provided:
- if no jobs are provided on input, then VROOM creates generical 
  job-entries with no coordinates.


# Output

The computed solution is written as `json` on standard output or a file
(using `-o`), formatted as follow.

| Key         | Description |
| ----------- | ----------- |
| `code` | return code, `0` if no error was raised |
| `error` | error message (present iff `code` is different from `0`) |
| [`summary`](#summary) | object summarizing solution indicators |
| [`routes`](#routes) | array of `route` objects |

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

## Summary

The `summary` object has the following properties:

| Key         | Description |
| ----------- | ----------- |
| `cost` | total cost for all routes |
| `duration`* | total duration in seconds for all routes |
| `distance`* | total distance in meters for all routes |
| [`computing_times`](#computing-times) | details for run-time information |

*: provided when using the `-g` flag with `OSRM`.

### Computing times

The `computing_times` object is used to report execution times in
milliseconds.

| Key | Description |
| ----------- | ----------- |
| `loading` | time required to parse the problem, compute and load the cost matrix |
| `solving` | time required to apply the solving strategy |
| `routing`* | time required to retrieve the detailed route geometries |

*: provided when using the `-g` flag with `OSRM`.

# Example

The file `input.json` describes a (very) small problem:

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

Running `vroom -i input.json -g` will provide a solution that looks
like:

```javascript
{
  "code": 0,
  "routes": [
    {
      "vehicle": 0,
      "cost": 1009,
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
      "geometry": "o`fiHaqjMBSPHh@T^PHDTLLHLFJFLFlBz@r@VXL\\NJDHDlCjAJDHDCPCNCNl\n@dGBVFp@Oz@bBt@RHPHtDhBHDJD@@bD~AnB~@RJLFFDbAd@PJTH`Ad@LFCHGZSpAO~@kApGOn@K^?BSf@Yv@k@zAw@nBGPwBnDOTmBrCa\n@l@W\\{@fBUd@}BvE{@hBW|ASvAe@jDAP?DJtC@\\Ab@qArIG\\UhAWlA_@fBoBnIEREP]zAMh@Kb@e@vBo@vCS|@oBvJADI\\GZEPy\n@dEGXKf@kFtWI`@Ml@GXI^?BOr@Mr@CN[zA?f@KzADbNBx@D`EBt@@h@?v@?RAFAr@@jDFr@\\fDFh@@`@@f@@J?p@B|CPpVFnL?L\n?`A?|AGb@?F?\\BbAF`A?D?HDtAFfBLjBV|BD\\L~@X|A`@nB~@|C`AfCfAhBLT`@n@fAzBtCtEhBvCtCxEt@|@bAfAvAgCwAfCIm\n@{@_BcEyGW_@S[QNK\\a@~@aErHCBGN_AoAIMyDoFqBmCm@w@a@wAw@oDo@aDGYGSCK?ECGO_AcB{MW}B[yEw@{JQiBEc@Ca@Ai@\n?kACsDCaFKgACoFIiKCkE?g@C_EAg@CuD?g@CiDIgMC_CAwBBsANaAXkA^gAn@sCXqAJk@hAoEzCcN^cBj@eCLg@`@kBViAhAcFlBqIzCyLhE_\n[|@oKt@uJPyAL_@BMJe@|@kINkAlAkGxAaI`@wC^kBLq@Ji@lAuGTmAx@qCv@kDZsALa@b@u@p@}BRkAp@yBd@oA~@eCJY\\aAPi\n@`@sAp@_DZ_BF[x@iDZy@?_@n@sCDc@US|@iDd@eBDOLa@L]v@sAj@_@ZIpAEb@APAJ@HAe@gCG_@[uAEWy@{DgBsIEOKe@Ow@Q{\n@WqAKi@YsA]}AKm@COGe@Cg@Am@HWFY@Q?SAQGYIWOQSKSAMBKDEBILEBKVCNAHAR?P@RFVBJDJNP@?PHIpAEj@IbAIfAATQ~Be@\n`D?F[bBKp@CLERERu@fDCNGRCHg@rBENMd@[`A_@jAQd@G`@Il@Mz@q@rEQnA[tBEVG\\c@vCCPCRc@tCStACPET_@fCQjACNQjAu\n@bFGMGGECuAq@IGERmAtEELKGs@c@CCeCwAIEIEqA}@GEcAs@OMIImAoAEHWl@Sv@U`BCVQx@CL`Br@",
      "distance": 14615,
      "duration": 1009
    }
  ],
  "solution": {
    "cost": 1009,
    "duration": 1009,
    "distance": 14615,
    "computing_times": {
      "routing": 7,
      "solving": 0,
      "loading": 13
    }
  }
}
```


The following input makes use of the option to provide a custom matrix:

```javascript
{
  "vehicles": [
    {
      "id":0,
      "start_index":0,
			"end_index":7
    }
  ],
  "matrix": [
    [0,2104,197,959,1975,436,418,1299],
    [2103,0,2255,2812,1455,2539,1685,3152],
    [197,2256,0,762,2172,633,570,1102],
    [959,2813,762,0,2934,1395,1127,340],
    [1975,1456,2172,2934,0,2020,2169,3274],
    [435,2539,632,1394,2019,0,853,1734],
    [418,1686,570,1127,2169,854,0,1467],
    [1299,3153,1102,340,3274,1735,1467,0] 
  ]
}
```

Generating the following output:

```javascript
{
  "code": 0,
  "summary": {
    "cost": 7268,
    "computing_times": {
      "loading": 0,
      "solving": 2
    }
  },
  "routes": [
    {
      "vehicle": 0,
      "cost": 7268,
      "steps": [
        {
          "type": "start",
          "start": 0
        },
        {
          "type": "job",
          "job": 5
        },
        {
          "type": "job",
          "job": 4
        },
        {
          "type": "job",
          "job": 1
        },
        {
          "type": "job",
          "job": 6
        },
        {
          "type": "job",
          "job": 2
        },
        {
          "type": "job",
          "job": 3
        },
        {
          "type": "end",
          "end": 7
        }
      ]
    }
  ]
}
```