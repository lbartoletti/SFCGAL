## Line sweep algorithm for segment intersections
## Bentley-Ottmann algorithm implementation

import geometry_types
import std/[algorithm, sets, tables, heapqueue]

type
  Segment* = object
    ## Line segment defined by two endpoints
    p1*, p2*: Coord2D
    id*: int  ## Segment identifier

  EventType = enum
    SegmentStart
    SegmentEnd
    Intersection

  Event = object
    point: Coord2D
    eventType: EventType
    segment: int  ## Index into segments array
    otherSegment: int  ## For intersection events

  Intersection* = object
    ## Intersection point with involved segments
    point*: Coord2D
    segment1*, segment2*: int

proc newSegment*(p1, p2: Coord2D, id: int = 0): Segment =
  ## Create a new segment, ensuring p1 < p2 lexicographically
  if p1 < p2:
    Segment(p1: p1, p2: p2, id: id)
  else:
    Segment(p1: p2, p2: p1, id: id)

proc `<`(a, b: Event): bool =
  ## Event ordering for priority queue (min-heap)
  ## Events ordered by x, then y, then by event type
  if a.point.x != b.point.x:
    return a.point.x > b.point.x  # Reverse for min-heap
  if a.point.y != b.point.y:
    return a.point.y > b.point.y  # Reverse for min-heap
  return ord(a.eventType) > ord(b.eventType)

proc orientation(p, q, r: Coord2D): float64 =
  ## Calculate orientation of ordered triplet (p, q, r)
  ## Returns: >0 for counter-clockwise, <0 for clockwise, 0 for collinear
  (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)

proc onSegment(p, q, r: Coord2D): bool =
  ## Check if point q lies on segment pr (given they are collinear)
  q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and
  q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y)

proc segmentsIntersect(s1, s2: Segment): bool =
  ## Check if two segments intersect
  let p1 = s1.p1
  let q1 = s1.p2
  let p2 = s2.p1
  let q2 = s2.p2

  let o1 = orientation(p1, q1, p2)
  let o2 = orientation(p1, q1, q2)
  let o3 = orientation(p2, q2, p1)
  let o4 = orientation(p2, q2, q1)

  # General case
  if o1 * o2 < 0 and o3 * o4 < 0:
    return true

  # Special cases - collinear points
  if o1 == 0.0 and onSegment(p1, p2, q1): return true
  if o2 == 0.0 and onSegment(p1, q2, q1): return true
  if o3 == 0.0 and onSegment(p2, p1, q2): return true
  if o4 == 0.0 and onSegment(p2, q1, q2): return true

  return false

proc computeIntersection(s1, s2: Segment): tuple[exists: bool, point: Coord2D] =
  ## Compute actual intersection point of two segments
  let p1 = s1.p1
  let p2 = s1.p2
  let p3 = s2.p1
  let p4 = s2.p2

  let x1 = p1.x
  let y1 = p1.y
  let x2 = p2.x
  let y2 = p2.y
  let x3 = p3.x
  let y3 = p3.y
  let x4 = p4.x
  let y4 = p4.y

  let denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

  if abs(denom) < 1e-10:
    return (false, newCoord2D(0, 0))

  let t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
  let u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom

  if t >= 0.0 and t <= 1.0 and u >= 0.0 and u <= 1.0:
    let x = x1 + t * (x2 - x1)
    let y = y1 + t * (y2 - y1)
    return (true, newCoord2D(x, y))

  return (false, newCoord2D(0, 0))

proc findIntersections*(segments: seq[Segment]): seq[Intersection] =
  ## Find all intersections between segments using sweep line algorithm
  result = @[]

  if segments.len < 2:
    return

  # Create events for all segment endpoints
  var events: HeapQueue[Event]

  for i, seg in segments:
    events.push(Event(point: seg.p1, eventType: SegmentStart, segment: i, otherSegment: -1))
    events.push(Event(point: seg.p2, eventType: SegmentEnd, segment: i, otherSegment: -1))

  # Status structure - segments intersecting sweep line
  # For simplicity, using a sequence (in production, use a balanced tree)
  var status: seq[int] = @[]

  # Process events
  while events.len > 0:
    let event = events.pop()

    case event.eventType
    of SegmentStart:
      # Add segment to status
      let segIdx = event.segment
      status.add(segIdx)

      # Check intersection with neighbors in status
      for i in 0..<status.len - 1:
        let otherIdx = status[i]
        if otherIdx != segIdx:
          if segmentsIntersect(segments[segIdx], segments[otherIdx]):
            let inter = computeIntersection(segments[segIdx], segments[otherIdx])
            if inter.exists:
              result.add(Intersection(
                point: inter.point,
                segment1: min(segIdx, otherIdx),
                segment2: max(segIdx, otherIdx)
              ))

    of SegmentEnd:
      # Remove segment from status
      let segIdx = event.segment
      status.keepItIf(it != segIdx)

    of Intersection:
      discard  # Already handled

  # Remove duplicate intersections
  var seen = initHashSet[tuple[x: float64, y: float64, s1: int, s2: int]]()
  var uniqueResult: seq[Intersection] = @[]

  for inter in result:
    let key = (inter.point.x, inter.point.y, inter.segment1, inter.segment2)
    if key notin seen:
      seen.incl(key)
      uniqueResult.add(inter)

  result = uniqueResult

proc findIntersectionsBruteForce*(segments: seq[Segment]): seq[Intersection] =
  ## Brute force intersection finding - O(n^2) for comparison
  result = @[]

  for i in 0..<segments.len:
    for j in i+1..<segments.len:
      if segmentsIntersect(segments[i], segments[j]):
        let inter = computeIntersection(segments[i], segments[j])
        if inter.exists:
          result.add(Intersection(
            point: inter.point,
            segment1: i,
            segment2: j
          ))
