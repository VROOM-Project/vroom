# Waste and materials transport: daily fleet planning problem

This document registers the real-world problem this fork of VROOM is
meant to address. It is a problem statement only; the analysis of how
(and whether) VROOM can solve it lives in
[waste_transport_vroom_analysis.md](./waste_transport_vroom_analysis.md).

Contents:
- [Context and goal](#context-and-goal)
- [Operations](#operations)
- [Containers](#containers)
- [Trucks](#trucks)
- [Loading rules](#loading-rules)
- [Constraints](#constraints)
- [Objective](#objective)
- [Dynamics: operations added during the day](#dynamics-operations-added-during-the-day)
- [Out of scope](#out-of-scope)
- [Non-functional requirements](#non-functional-requirements)
- [Open questions](#open-questions)

## Context and goal

A company transports waste containers and materials. Every morning
the planner has:

- a list of **operations** requested for the day (some may be
  carried over from previous days);
- a set of available **trucks** and **operators** (drivers).

The plan to produce is, for every truck: the ordered list of
operations it performs today and the route it drives. Operations that
do not fit today are left unassigned and move on to the next day's
planning. Assigning operators to trucks is done afterwards and is not
part of the optimisation.

The company site (referred to below as **the company**) is where the
trucks start and end the day, where full containers are emptied and
where empty containers and materials are stocked.

## Operations

Four kinds of operations exist. Each one is a visit to a location that
changes what the truck carries.

| Operation | Location | Effect on the truck load |
| --- | --- | --- |
| Pick up a waste container | client site | a **full** container of a given size is loaded |
| Leave a waste container somewhere | disposal site, client, or the company | a **full** container is unloaded |
| Leave an empty container somewhere | client site | an **empty** container of a given size is unloaded |
| Deliver materials | client site | materials are unloaded |

Operations are coupled through the truck load:

- a full container picked up at a client has to be brought somewhere:
  usually back to the company to be emptied, sometimes directly to
  another place ("pure transport": pick up at A and leave at B without
  passing through the company);
- an empty container left at a client has to be on board first; empty
  containers come from the company;
- because a truck can only carry a few containers at a time, a
  typical day is a sequence of **trips**: go out, pick up and/or leave
  containers, come back to the company to unload, go out again.

A single client visit may combine operations (for instance leave an
empty container and pick up the full one standing there).

From the planner's point of view, requests come in four types, which
the planning tool translates into the movements above:

| Request type | Movements |
| --- | --- |
| Deliver an empty container | company to client, empty container |
| Pick up an empty container | client to company, empty container |
| Exchange an empty container for a full one | empty container company to client, full container client to company |
| Sell materials | company to client, a full truck of materials, nothing else on board |

## Containers

Container sizes: **2 m³, 6 m³, 12 m³, 20 m³, 40 m³**.

A container is either **empty** or **full**. The state matters: full
containers are heavy and cannot be stacked on top of anything, empty
ones can be nested or placed on top of others. Throughout this
document `eN` means an empty N m³ container and `fN` a full one
(`3e2` = three empty 2 m³ containers, `1f6` = one full 6 m³
container).

## Trucks

Three truck types:

| Type | Containers it can handle |
| --- | --- |
| small | 2 m³ |
| multiban | 2, 6, 12 m³ |
| poliban | 20, 40 m³ |

The multiban and the poliban can be fitted with an **extender** that
makes them bigger. Which combinations become possible with the
extender is not specified yet (see [open questions](#open-questions)).

Compatibility between containers and trucks is strict: a truck can
only carry the container sizes of its type.

## Loading rules

What a truck can carry at the same time is **not** a simple sum of
volumes or a count. It is a fixed list of allowed combinations,
decided by how containers physically sit on the truck (which ones can
go on top of or inside which). The authoritative lists provided by
the company are kept in **one place**,
[waste_rules.json](./waste_rules.json), which the planner, the check
scripts and the example instance all read. Anything not listed there
is assumed forbidden until confirmed otherwise (see open questions).

A combination that is "smaller" than an allowed one is also allowed
(if `6e2` is allowed, so is `4e2`).

Notation used in that file: `eN` an empty N m³ container, `fN` a full
one, `+` to combine (`2f6 + 1e2` = two full 6 m³ and one empty 2 m³).
Each truck type has one list of maximal allowed loads.

### Physical arrangements (informative only)

The mixed full-and-empty loads of the multiban come from how
containers sit on the truck; this does not constrain the plan (see the
notes below) but explains the list:

| Multiban load | Physical arrangement |
| --- | --- |
| `1f6 + 3e2` | the 2 m³ on top of the full 6 m³ |
| `2f6 + 1e2` | the 2 m³ on top of one of the full 6 m³ |
| `1f6 + 3e6` | full one below, empties on top |
| `1f12 + 1e2` | the 2 m³ on top of the full 12 m³ |

### Notes on the rules

- The arrangement column explains where the rules come from but does
  **not** constrain the plan: when loading or unloading at a site the
  operator can reorder the containers on the truck as needed. Only the
  *set* of containers on board matters, never their order.
- Modelling the multiban capacity is the main modelling difficulty.
  Earlier attempts based on additive quantities (volume, count) were
  incorrect because they either allowed forbidden combinations or
  forbade allowed ones.

## Constraints

- **Working hours**: every truck has a start time and a finishing time
  for the day.
- **Lunch break**: every truck (operator) has a lunch break during the
  day.
- **Compatibility**: containers can only travel on trucks of a
  compatible type (see [Trucks](#trucks)).
- **Loading rules**: at every moment the load on a truck must be one
  of the allowed combinations for that truck (see
  [Loading rules](#loading-rules)).
- **Load conservation**: a container can only be left somewhere if it
  is on board; a full container picked up has to be unloaded somewhere
  (company or destination) before the end of the day.
- **Unassigned operations are acceptable**: if not everything fits in
  the day, the leftover moves to the next day. Operations can carry a
  **priority** so that the planner controls which ones are dropped
  first.

## Objective

A good plan, in order of importance:

1. maximises the number of operations done today (weighted by
   priority);
2. minimises the total time spent doing them;
3. if possible, also minimises the kilometres driven.

## Dynamics: operations added during the day

From time to time a new operation is requested while the trucks are
already out. It would be useful to be able to insert it into the
running plan: re-plan from the current situation (where each truck is,
what it carries, what it has already done) without discarding the
work already done. Travel times from the current truck positions are
obtained from OSRM at that moment.

## Out of scope

- Assigning operators to trucks (done manually afterwards).
- Anything happening inside the company site (queues at the
  unloading point, etc.).

## Non-functional requirements

- The fleet is small; latency is not an issue. A planning run taking
  several minutes is acceptable.
- Routing data (travel times and distances between locations) comes
  from an OSRM instance running alongside the planner. It is also what
  makes re-planning during the day possible: travel times from the
  current truck positions are obtained from OSRM on demand.

## Open questions

Points that need confirmation from the company before the model can
be finalised. They matter because the answer changes whether the rules
can be expressed exactly (see the analysis document).

1. **Unlisted mixtures on the multiban.** Several combinations that
   sit "between" listed ones are not in the list. Are they allowed?
   - `3e2 + 3e6` (empties of both small sizes together)
   - `1f2 + 1f6` (one full of each)
   - `1f2 + 3e2` (analogous to `1f6 + 3e2`)
   - `1e12 + 3e2` (analogous to `1e12 + 3e6`)
   - `1f12 + 1e6` (analogous to `1f12 + 1e2`)
   - `2e12` (two empty 12 m³ nested)
2. **`1f12 + 1f2`.** One description says a full 12 m³ can travel with
   a 2 m³ "full or empty"; the final list only has `1f12 + 1e2`. Which
   is right?
3. **Extender.** What does each truck type carry with the extender
   fitted? Is the decision to fit it taken per truck per day (an input
   to the planner) or should the planner decide?
4. **Materials.** How are materials transported: inside a container
   (which size, on which truck) or loose in the truck? Do they share
   the truck with containers?
5. **Small truck with a full 2 m³.** Is `1f2 + 1e2` or `1f2 + 2e2`
   allowed on the small truck, or is a full 2 m³ always alone?
6. **Where empties come from.** Always from the company, or can an
   empty container unloaded at a client have been picked up empty
   elsewhere?
7. **Disposal sites.** When a full container is "left somewhere" other
   than the company, is that a final drop (the container stays there)
   or does the truck wait and bring the container back?
8. **Unloading at the company.** Typical duration of a company visit
   (fixed part plus a per-container part), and whether the company has
   opening hours that constrain it.
