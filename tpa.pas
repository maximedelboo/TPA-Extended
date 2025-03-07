unit tpa;
{==============================================================================]
  Copyright:
   - Copyright (c) 2016, Jarl `slacky` Holta
   - Raymond van Venetië and Merlijn Wajer
  License: GNU General Public License (https://www.gnu.org/licenses/gpl-3.0)
  Links:
   - https://github.com/Torwent/Simba/blob/simba1400/Source/MML/simba.tpa.pas
   - https://github.com/Villavu/Simba/blob/simba2000/Source/simba.vartype_pointarray.pas
   - https://pastebin.com/8hxwnptq
[==============================================================================}
{$mode objfpc}{$H+}

interface

uses sysutils, types;

function NRSplitTPA(const pts: TPointArray;  w,h: Single): T2DPointArray;
function NRClusterTPA(const tpa: TPointArray; dist: Single): T2DPointArray;
function SkeletonTPA(tpa: TPointArray; fMin, fMax: Int32): TPointArray;
function TPAMatrix(tpa: TPointArray): T2DBoolArray;

type TNode = record Pt: TPoint; Weight: Int32; end;
type TQueue = array of TNode;

procedure _SiftDown(var queue: TQueue; startpos, pos: Int32);
procedure _SiftUp(var queue: TQueue; pos: Int32);

type TAStarNodeData = record Parent: TPoint; Open, Closed: Boolean; ScoreA, ScoreB: Int32; end;
type TAStarData = array of array of TAStarNodeData;

procedure _Push(var queue: TQueue; node: TNode; var data: TAStarData; var size: Int32);
function _Pop(var queue: TQueue; var data: TAStarData; var size: Int32): TNode;
function _BuildPath(start, goal: TPoint; data: TAStarData; offset: TPoint): TPointArray;

function AStarTPAEx(tpa: TPointArray; out paths: T2DFloatArray; start, goal: TPoint; diagonalTravel: Boolean): TPointArray;

function AStarTPA(tpa: TPointArray; start, goal: TPoint; diagonalTravel: Boolean): TPointArray; overload;

procedure _GetBoundingBox(const tpa: TPointArray; out MinX, MinY, MaxX, MaxY: Integer);
procedure _FloodFill(const grid: T2DBoolArray; var visited: T2DBoolArray; OffX, OffY, startX, startY: Integer; var cluster: TPointArray);
function FloodFillTPA(const tpa: TPointArray): T2DPointArray;

implementation

uses Math;


function NRSplitTPA(const pts: TPointArray; w,h: Single): T2DPointArray;
var
  bak: TPoint;
  lo,hi: Int32;
  sqx,sqy,sqxy: single;
  procedure ConnectCluster(p: TPoint);
  var i,top: Int32;
  begin
    top := hi;
    for i:=hi downto lo+1 do
      //if (Abs(p.x-pts[i].x) <= w) and (Abs(p.y-pts[i].y) <= h) then
      if (Sqr(p.x-pts[i].x)*sqy)+(Sqr(p.y-pts[i].y)*sqx) <= sqxy then
      begin
        bak := pts[i];
        pts[i] := pts[hi];
        pts[hi] := bak;
        Dec(hi);               // reduce upper bound
      end;

    for i:=hi+1 to top do
      ConnectCluster(pts[i]);
  end;

var top,n: Int32;
begin
  sqx := Sqr(w);
  sqy := Sqr(h);
  sqxy := sqx*sqy;

  lo := 0;
  hi := High(pts);
  while lo <= hi do
  begin
    top := hi;
    ConnectCluster(pts[lo]);

    // add the connected range
    n := Length(Result);
    SetLength(Result, n+1);
    Result[n] := Copy(pts, hi+1, (top-hi));
    SetLength(Result[n], Length(Result[n]) + 1);
    Result[n][High(Result[n])] := pts[lo];

    Inc(lo); // increase lower bound
  end;
end;

function NRClusterTPA(const tpa: TPointArray; dist: Single): T2DPointArray;
type
  TPointScan = record
    skipRow: Boolean;
    count: Integer;
  end;
var
  h, i, l, c, s, x, y, o, r, d, m: Integer;
  p: array of array of TPointScan;
  q: TPointArray;
  a, b, t: TBox;
  e: Extended;
  z: TPoint;
  v: Boolean;
begin
  SetLength(Result, 0);
  h := High(TPA);
  if (h > -1) then
    if (h > 0) then
    begin
      b.X1 := TPA[0].X;
      b.Y1 := TPA[0].Y;
      b.X2 := TPA[0].X;
      b.Y2 := TPA[0].Y;
      r := 0;
      for i := 1 to h do
      begin
        if (TPA[i].X < b.X1) then
          b.X1 := TPA[i].X
        else
          if (TPA[i].X > b.X2) then
            b.X2 := TPA[i].X;
        if (TPA[i].Y < b.Y1) then
          b.Y1 := TPA[i].Y
        else
          if (TPA[i].Y > b.Y2) then
            b.Y2 := TPA[i].Y;
      end;
      SetLength(p, ((b.X2 - b.X1) + 1));
      for i := 0 to (b.X2 - b.X1) do
      begin
        SetLength(p[i], ((b.Y2 - b.Y1) + 1));
        for c := 0 to (b.Y2 - b.Y1) do
        begin
          p[i][c].count := 0;
          p[i][c].skipRow := False;
        end;
      end;
      e := dist;
      if (e < 0.0) then
        e := 0.0;
      d := Ceil(e);
      m := Max(((b.X2 - b.X1) + 1), ((b.Y2 - b.Y1) + 1));
      if (d > m) then
        d := m;
      for i := 0 to h do
        Inc(p[(TPA[i].X - b.X1)][(TPA[i].Y - b.Y1)].count);
      for i := 0 to h do
        if (p[(TPA[i].X - b.X1)][(TPA[i].Y - b.Y1)].count > 0) then
        begin
          c := Length(Result);
          SetLength(Result, (c + 1));
          SetLength(Result[c], p[(TPA[i].X - b.X1)][(TPA[i].Y - b.Y1)].count);
          for o := 0 to (p[(TPA[i].X - b.X1)][(TPA[i].Y - b.Y1)].count - 1) do
            Result[c][o] := TPA[i];
          r := (r + p[(TPA[i].X - b.X1)][(TPA[i].Y - b.Y1)].count);
          if (r > h) then
            Exit;
          SetLength(q, 1);
          q[0] := TPA[i];
          p[(TPA[i].X - b.X1)][(TPA[i].Y - b.Y1)].count := 0;
          s := 1;
          while (s > 0) do
          begin
            s := High(q);
            z := q[s];
            a.X1 := (z.X - d);
            a.Y1 := (z.Y - d);
            a.X2 := (z.X + d);
            a.Y2 := (z.Y + d);
            t := a;
            SetLength(q, s);
            if (a.X1 < b.X1) then
              a.X1 := b.X1
            else
              if (a.X1 > b.X2) then
                a.X1 := b.X2;
            if (a.Y1 < b.Y1) then
              a.Y1 := b.Y1
            else
              if (a.Y1 > b.Y2) then
                a.Y1 := b.Y2;
            if (a.X2 < b.X1) then
              a.X2 := b.X1
            else
              if (a.X2 > b.X2) then
                a.X2 := b.X2;
            if (a.Y2 < b.Y1) then
              a.Y2 := b.Y1
            else
              if (a.Y2 > b.Y2) then
                a.Y2 := b.Y2;
            case ((t.X1 <> a.X1) or (t.X2 <> a.X2)) of
              True:
              for y := a.Y1 to a.Y2 do
                if not p[(a.X2 - b.X1)][(y - b.Y1)].skipRow then
                for x := a.X1 to a.X2 do
                  if (p[(x - b.X1)][(y - b.Y1)].count > 0) then
                    if (Sqrt(Sqr(z.X - x) + Sqr(z.Y - y)) <= dist) then
                    begin
                      l := Length(Result[c]);
                      SetLength(Result[c], (l + p[(x - b.X1)][(y - b.Y1)].count));
                      for o := 0 to (p[(x - b.X1)][(y - b.Y1)].count - 1) do
                      begin
                        Result[c][(l + o)].X := x;
                        Result[c][(l + o)].Y := y;
                      end;
                      r := (r + p[(x - b.X1)][(y - b.Y1)].count);
                      if (r > h) then
                        Exit;
                      p[(x - b.X1)][(y - b.Y1)].count := 0;
                      SetLength(q, (s + 1));
                      q[s] := Result[c][l];
                      Inc(s);
                    end;
              False:
              for y := a.Y1 to a.Y2 do
                if not p[(a.X2 - b.X1)][(y - b.Y1)].skipRow then
                begin
                  v := True;
                  for x := a.X1 to a.X2 do
                    if (p[(x - b.X1)][(y - b.Y1)].count > 0) then
                      if (Sqrt(Sqr(z.X - x) + Sqr(z.Y - y)) <= dist) then
                      begin
                        l := Length(Result[c]);
                        SetLength(Result[c], (l + p[(x - b.X1)][(y - b.Y1)].count));
                        for o := 0 to (p[(x - b.X1)][(y - b.Y1)].count - 1) do
                        begin
                          Result[c][(l + o)].X := x;
                          Result[c][(l + o)].Y := y;
                        end;
                        r := (r + p[(x - b.X1)][(y - b.Y1)].count);
                        if (r > h) then
                          Exit;
                        p[(x - b.X1)][(y - b.Y1)].count := 0;
                        SetLength(q, (s + 1));
                        q[s] := Result[c][l];
                        Inc(s);
                      end else
                        v := False;
                  if v then
                    p[(a.X2 - b.X1)][(y - b.Y1)].skipRow := True;
                end;
            end;
          end;
        end;
    end else
    begin
      SetLength(Result, 1);
      SetLength(Result[0], 1);
      Result[0][0] := TPA[0];
    end;
end;

function SkeletonTPA(tpa: TPointArray; fMin, fMax: Int32): TPointArray;
  function _TransitCount(const p2,p3,p4,p5,p6,p7,p8,p9: Int32): Int32;
  begin
    Result := 0;

    if ((p2 = 0) and (p3 = 1)) then Inc(Result);
    if ((p3 = 0) and (p4 = 1)) then Inc(Result);
    if ((p4 = 0) and (p5 = 1)) then Inc(Result);
    if ((p5 = 0) and (p6 = 1)) then Inc(Result);
    if ((p6 = 0) and (p7 = 1)) then Inc(Result);
    if ((p7 = 0) and (p8 = 1)) then Inc(Result);
    if ((p8 = 0) and (p9 = 1)) then Inc(Result);
    if ((p9 = 0) and (p2 = 1)) then Inc(Result);
  end;

var
  j,i,x,y,h,transit,sumn,MarkHigh,hits: Int32;
  p2,p3,p4,p5,p6,p7,p8,p9:Int32;
  Change, PTS: TPointArray;
  Matrix: array of TBoolArray;
  iter: Boolean;
  B: TBox;
begin
  h := High(tpa);
  if (h <= 0) then
    Exit;

  B := GetTPABounds(tpa);
  B.X1 -= 2;
  B.Y1 -= 2;

  SetLength(Matrix, (B.Y2 - B.Y1 + 1) + 2, (B.X2 - B.X1 + 1) + 2);
  SetLength(PTS, h + 1);

  for i := 0 to h do
  begin
    x := tpa[i].X - B.X1;
    y := tpa[i].Y - B.Y1;
    PTS[i] := Point(x, y);
    Matrix[y, x] := True;
  end;

  j := 0;
  MarkHigh := h;
  SetLength(Change, h+1);

  repeat
    iter := (j mod 2) = 0;
    Hits := 0;
    i := 0;
    while (i < MarkHigh) do
    begin
      x := PTS[i].x;
      y := PTS[i].y;
      p2 := Ord(Matrix[y-1,x]);
      p4 := Ord(Matrix[y,x+1]);
      p6 := Ord(Matrix[y+1,x]);
      p8 := Ord(Matrix[y,x-1]);

      if iter then
      begin
        if (((p4 * p6 * p8) <> 0) or ((p2 * p4 * p6) <> 0)) then
        begin
          Inc(i);
          Continue;
        end;
      end
      else if ((p2 * p4 * p8) <> 0) or ((p2 * p6 * p8) <> 0) then
      begin
        Inc(i);
        Continue;
      end;

      p3 := Ord(Matrix[y-1,x+1]);
      p5 := Ord(Matrix[y+1,x+1]);
      p7 := Ord(Matrix[y+1,x-1]);
      p9 := Ord(Matrix[y-1,x-1]);
      Sumn := (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9);
      if (SumN >= FMin) and (SumN <= FMax) then
      begin
        Transit := _TransitCount(p2,p3,p4,p5,p6,p7,p8,p9);
        if (Transit = 1) then
        begin
          Change[Hits] := PTS[i];
          Inc(Hits);
          PTS[i] := PTS[MarkHigh];
          PTS[MarkHigh] := Point(x, y);
          Dec(MarkHigh);
          Continue;
        end;
      end;
      Inc(i);
    end;

    for i:=0 to (Hits-1) do
      Matrix[Change[i].y, Change[i].x] := False;

    inc(j);
  until ((Hits=0) and (Iter=False));

  SetLength(Result, (MarkHigh + 1));
  for i := 0 to MarkHigh do
    Result[i] := Point(PTS[i].X+B.X1, PTS[i].Y+B.Y1);
end;

function TPAMatrix(tpa: TPointArray): T2DBoolArray;
var
  b: TBox;
  p: TPoint;
begin
  b := GetTPABounds(tpa);
  SetLength(Result, b.Y2+1, b.X2+1);
  for p in tpa do Result[p.Y, p.X] := True;
end;


//AStar
procedure _SiftDown(var queue: TQueue; startpos, pos: Int32);
var
  parentpos: Int32;
  parent,newitem: TNode;
begin
  newitem := queue[pos];
  while pos > startpos do
  begin
    parentpos := (pos - 1) shr 1;
    parent := queue[parentpos];
    if (newitem.Weight < parent.Weight) then
    begin
      queue[pos] := parent;
      pos := parentpos;
      continue;
    end;
    Break;
  end;
  queue[pos] := newitem;
end;

procedure _SiftUp(var queue: TQueue; pos: Int32);
var
  endpos, startpos, childpos, rightpos: Int32;
  newitem: TNode;
begin
  endpos := Length(queue);
  startpos := pos;
  newitem := queue[pos];
  // Move the smaller child up until hitting a leaf.
  childpos := 2 * pos + 1;    // leftmost child
  while (childpos < endpos) do
  begin
    // Set childpos to index of smaller child.
    rightpos := childpos + 1;
    if (rightpos < endpos) and (queue[childpos].Weight >= queue[rightpos].Weight) then
      childpos := rightpos;
    // Move the smaller child up.
    queue[pos] := queue[childpos];
    pos := childpos;
    childpos := 2 * pos + 1;
  end;
  // This (`pos`) node/leaf is empty. So we can place "newitem" in here, then
  // push it up to its final place (by sifting its parents down).
  queue[pos] := newitem;
  _SiftDown(queue, startpos, pos);
end;

procedure _Push(var queue: TQueue; node: TNode; var data: TAStarData; var size: Int32);
var
  i: Int32;
begin
  i := Length(queue);
  SetLength(queue, i + 1);
  queue[i] := node;
  _SiftDown(queue, 0, i);
  data[node.Pt.Y, node.Pt.X].Open := True;
  Inc(size);
end;

function _Pop(var queue: TQueue; var data: TAStarData; var size: Int32): TNode;
var
  node: TNode;
begin
  node := queue[High(queue)];
  SetLength(queue, High(queue));

  if Length(queue) > 0 then
  begin
    Result := queue[0];
    queue[0] := node;
    _SiftUp(queue, 0);
  end
  else
    Result := node;

  data[Result.Pt.Y, Result.Pt.X].Open := False;
  data[Result.Pt.Y, Result.Pt.X].Closed := True;
  Dec(size);
end;

function _BuildPath(start, goal: TPoint; data: TAStarData; offset: TPoint): TPointArray;
var
  tmp: TPoint;
  len: Int32 = 0;
begin
  tmp := goal;

  while tmp <> start do
  begin
    Inc(len);
    SetLength(Result, len);
    Result[len-1].X := tmp.X + offset.X;
    Result[len-1].Y := tmp.Y + offset.Y;
    tmp := data[tmp.Y, tmp.X].Parent;
  end;

  Inc(len);
  SetLength(Result, len);
  Result[len-1].X := tmp.X + offset.X;
  Result[len-1].Y := tmp.Y + offset.Y;
  TPAReverse(Result);
end;


function AStarTPAEx(tpa: TPointArray; out paths: T2DFloatArray; start, goal: TPoint; diagonalTravel: Boolean): TPointArray;
const
  OFFSETS: array[0..7] of TPoint = ((X:0; Y:-1),(X:-1; Y:0),(X:1; Y:0),(X:0; Y:1),(X:1; Y:-1),(X:-1; Y:1),(X:1; Y:1),(X:-1; Y:-1));
var
  b: TBox;
  queue: TQueue;
  data: TAStarData;
  matrix: T2DBoolArray;
  score, i, hi, size: Int32;
  node: TNode;
  offset, q, p: TPoint;
begin
  b := GetTPABounds(tpa);
  if not b.Contains(start) then Exit;
  if not b.Contains(goal) then Exit;

  offset.X := b.X1;
  offset.Y := b.Y1;
  start.X -= offset.X;
  start.Y -= offset.Y;
  goal.X -= offset.X;
  goal.Y -= offset.Y;

  b.X1 := 0;
  b.Y1 := 0;
  b.X2 -= offset.X;
  b.Y2 -= offset.Y;

  SetLength(matrix, b.Y2+1, b.X2+1);
  for p in tpa do
    matrix[p.Y - offset.Y, p.X - offset.X] := True;

  if not matrix[start.Y, start.X] then Exit;
  if not matrix[goal.Y, goal.X] then Exit;

  SetLength(paths, 0);
  SetLength(paths, offset.Y + b.Y2+1, offset.X + b.X2+1);
  SetLength(data, b.Y2+1, b.X2+1);

  data[start.Y, start.X].ScoreB := Sqr(start.X - goal.X) + Sqr(start.Y - goal.Y);

  node.Pt := start;
  node.Weight := data[start.Y, start.X].ScoreB;
  _Push(queue, node, data, size);

  if diagonalTravel then hi := 7 else hi := 3;

  while (size > 0) do
  begin
    node := _Pop(queue, data, size);
    p := node.Pt;

    if p = goal then Exit(_BuildPath(start, goal, data, offset));

    for i := 0 to hi do
    begin
      q := p + OFFSETS[i];

      if not b.Contains(q) then Continue;
      if not matrix[q.Y, q.X] then Continue;

      score := data[p.Y, p.X].ScoreA + 1;

      if data[q.Y, q.X].Closed and (score >= data[q.Y, q.X].ScoreA) then
        Continue;
      if data[q.Y, q.X].Open and (score >= data[q.Y, q.X].ScoreA) then
        Continue;

      data[q.Y, q.X].Parent := p;
      data[q.Y, q.X].ScoreA := score;
      data[q.Y, q.X].ScoreB := data[q.Y, q.X].ScoreA + Sqr(q.X - goal.X) + Sqr(q.Y - goal.Y);;

      if data[q.Y, q.X].Open then Continue;

      paths[q.Y + offset.Y, q.X + offset.X] := score;

      node.Pt := q;
      node.Weight := data[q.Y, q.X].ScoreB;
      _Push(queue, node, data, size);
    end;
  end;


  Result := [];
end;

function AStarTPA(tpa: TPointArray; start, goal: TPoint; diagonalTravel: Boolean): TPointArray;
const
  OFFSETS: array[0..7] of TPoint = ((X:0; Y:-1),(X:-1; Y:0),(X:1; Y:0),(X:0; Y:1),(X:1; Y:-1),(X:-1; Y:1),(X:1; Y:1),(X:-1; Y:-1));
var
  b: TBox;
  queue: TQueue;
  data: TAStarData;
  matrix: T2DBoolArray;
  score, i, hi, size: Int32;
  node: TNode;
  offset, q, p: TPoint;
begin
  b := GetTPABounds(tpa);
  if not b.Contains(start) then Exit;
  if not b.Contains(goal) then Exit;

  offset.X := b.X1;
  offset.Y := b.Y1;
  start.X -= offset.X;
  start.Y -= offset.Y;
  goal.X -= offset.X;
  goal.Y -= offset.Y;

  b.X1 := 0;
  b.Y1 := 0;
  b.X2 -= offset.X;
  b.Y2 -= offset.Y;

  SetLength(matrix, b.Y2+1, b.X2+1);
  for p in tpa do
    matrix[p.Y - offset.Y, p.X - offset.X] := True;

  if not matrix[start.Y, start.X] then Exit;
  if not matrix[goal.Y, goal.X] then Exit;

  SetLength(data, b.Y2 + 1, b.X2 + 1);

  data[start.Y, start.X].ScoreB := Sqr(start.X - goal.X) + Sqr(start.Y - goal.Y);

  node.Pt := start;
  node.Weight := data[start.Y, start.X].ScoreB;
  _Push(queue, node, data, size);

  if diagonalTravel then hi := 7 else hi := 3;

  while (size > 0) do
  begin
    node := _Pop(queue, data, size);
    p := node.Pt;

    if p = goal then Exit(_BuildPath(start, goal, data, offset));

    for i := 0 to hi do
    begin
      q := p + OFFSETS[i];

      if not b.Contains(q) then Continue;
      if not matrix[q.Y, q.X] then Continue;

      score := data[p.Y, p.X].ScoreA + 1;

      if data[q.Y, q.X].Closed and (score >= data[q.Y, q.X].ScoreA) then
        Continue;
      if data[q.Y, q.X].Open and (score >= data[q.Y, q.X].ScoreA) then
        Continue;

      data[q.Y, q.X].Parent := p;
      data[q.Y, q.X].ScoreA := score;
      data[q.Y, q.X].ScoreB := data[q.Y, q.X].ScoreA + Sqr(q.X - goal.X) + Sqr(q.Y - goal.Y);;

      if data[q.Y, q.X].Open then Continue;

      node.Pt := q;
      node.Weight := data[q.Y, q.X].ScoreB;
      _Push(queue, node, data, size);
    end;
  end;

  Result := [];
end;


procedure _GetBoundingBox(const tpa: TPointArray; out MinX, MinY, MaxX, MaxY: Integer);
var
  i: Integer;
begin
  if Length(tpa) = 0 then
    raise Exception.Create('Empty point array');
  MinX := tpa[0].X;
  MaxX := tpa[0].X;
  MinY := tpa[0].Y;
  MaxY := tpa[0].Y;
  for i := 1 to High(tpa) do
  begin
    if tpa[i].X < MinX then MinX := tpa[i].X;
    if tpa[i].X > MaxX then MaxX := tpa[i].X;
    if tpa[i].Y < MinY then MinY := tpa[i].Y;
    if tpa[i].Y > MaxY then MaxY := tpa[i].Y;
  end;
end;

procedure _FloodFill(const grid: T2DBoolArray; var visited: T2DBoolArray; OffX, OffY, startX, startY: Integer; var cluster: TPointArray);
var
  // Queue for BFS.
  queue: array of TPoint;
  front, rear: Integer;
  cur, neighbor: TPoint;
  dx, dy: Integer;
  nx, ny: Integer;
  // 4-connected neighbor offsets.
  Neighbors: array[0..3] of TPoint = (
    (X: 0; Y: -1),  // Up
    (X: 0; Y: 1),   // Down
    (X: -1; Y: 0),  // Left
    (X: 1; Y: 0)    // Right
  );
  i: Integer;
  width, height: Integer;
begin
  width := Length(grid);
  if width = 0 then Exit;
  height := Length(grid[0]);

  SetLength(queue, width * height);
  front := 0;
  rear := 0;

  // Start from the given cell.
  queue[rear].X := startX;
  queue[rear].Y := startY;
  Inc(rear);
  visited[startX][startY] := True;
  // Add the corresponding point (convert grid indices back to real coordinates)
  SetLength(cluster, Length(cluster) + 1);
  cluster[High(cluster)].X := startX + OffX;
  cluster[High(cluster)].Y := startY + OffY;

  // Process queue
  while front < rear do
  begin
    cur := queue[front];
    Inc(front);

    // Check neighbors
    for i := Low(Neighbors) to High(Neighbors) do
    begin
      dx := Neighbors[i].X;
      dy := Neighbors[i].Y;
      nx := cur.X + dx;
      ny := cur.Y + dy;

      // Check bounds.
      if (nx >= 0) and (nx < width) and (ny >= 0) and (ny < height) then
      begin
        // If the neighbor is occupied and not visited
        if (grid[nx][ny]) and (not visited[nx][ny]) then
        begin
          visited[nx][ny] := True;
          // Enqueue
          queue[rear].X := nx;
          queue[rear].Y := ny;
          Inc(rear);
          // Add to cluster (convert back to original coordinates)
          SetLength(cluster, Length(cluster) + 1);
          cluster[High(cluster)].X := nx + OffX;
          cluster[High(cluster)].Y := ny + OffY;
        end;
      end;
    end;
  end;
end;

function FloodFillTPA(const tpa: TPointArray): T2DPointArray;
var
  MinX, MinY, MaxX, MaxY: Integer;
  width, height: Integer;
  grid, visited: T2DBoolArray;
  i, j: Integer;
  pt: TPoint;
  cluster: TPointArray;
  clusters: T2DPointArray;
begin
  _GetBoundingBox(tpa, MinX, MinY, MaxX, MaxY);
  width := MaxX - MinX + 1;
  height := MaxY - MinY + 1;

  SetLength(grid, width);
  SetLength(visited, width);
  for i := 0 to width - 1 do
  begin
    SetLength(grid[i], height);
    SetLength(visited[i], height);
    FillChar(grid[i][0], height * SizeOf(Boolean), 0);
    FillChar(visited[i][0], height * SizeOf(Boolean), 0);
  end;

  for i := 0 to High(tpa) do
  begin
    pt := tpa[i];
    grid[pt.X - MinX][pt.Y - MinY] := True;
  end;

  clusters := nil;

  for i := 0 to width - 1 do
    for j := 0 to height - 1 do
    begin
      pt.Y := j;
      if grid[i][pt.Y] and (not visited[i][pt.Y]) then
      begin
        // Start a new cluster
        cluster := nil;
        _FloodFill(grid, visited, MinX, MinY, i, pt.Y, cluster);
        // Append cluster to clusters if non-empty.
        if Length(cluster) > 0 then
        begin
          SetLength(clusters, Length(clusters) + 1);
          clusters[High(clusters)] := cluster;
        end;
      end;
    end;

  Result := clusters;
end;

end.

