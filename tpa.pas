unit tpa;
{==============================================================================]
  Copyright:
   - Copyright (c) 2016, Jarl `slacky` Holta
   - Raymond van Venetië and Merlijn Wajer
  License: GNU General Public License (https://www.gnu.org/licenses/gpl-3.0)
  Links:
   - https://github.com/Torwent/Simba/blob/simba1400/Source/MML/simba.tpa.pas
   - https://github.com/Villavu/Simba/blob/simba2000/Source/simba.vartype_pointarray.pas
[==============================================================================}
{$mode objfpc}{$H+}

interface

uses sysutils, types;

function NRSplitTPA(const arr: TPointArray; dist: Double): T2DPointArray;
function NRClusterTPA(const tpa: TPointArray; dist: Double): T2DPointArray;
function SkeletonTPA(tpa: TPointArray; fMin, fMax: Int32): TPointArray;
function TPAMatrix(tpa: TPointArray): T2DBoolArray;
function AStarTPA(tpa: TPointArray; start, goal: TPoint; out totalDist: Double; diagonalTravel: Boolean = True; maxDistanceMultiplier: Int32 = 5): TPointArray;

implementation

uses Math;


function NRSplitTPA(const arr: TPointArray; dist: Double): T2DPointArray;
var
  t1, t2, c, ec, tc, l: Integer;
  tpa: TPointArray;
begin
  tpa := Copy(arr);
  l := High(tpa);
  if (l < 0) then Exit;
  SetLength(Result, l + 1);
  c := 0;
  ec := 0;
  while ((l - ec) >= 0) do
  begin
    SetLength(Result[c], 1);
    Result[c][0] := tpa[0];
    tpa[0] := tpa[l - ec];
    Inc(ec);
    tc := 1;
    t1 := 0;
    while (t1 < tc) do
    begin
      t2 := 0;
      while (t2 <= (l - ec)) do
      begin
        if (sqrt(Sqr(Result[c][t1].x - tpa[t2].x) + Sqr(Result[c][t1].y - tpa[t2].y)) <= dist) then
        begin
          SetLength(Result[c], tc +1);
          Result[c][tc] := tpa[t2];
          tpa[t2] := tpa[l - ec];
          Inc(ec);
          Inc(tc);
          Dec(t2);
        end;
        Inc(t2);
      end;
      Inc(t1);
    end;
    Inc(c);
  end;
  SetLength(Result, c);
end;

function NRClusterTPA(const tpa: TPointArray; dist: Double): T2DPointArray;
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

  B := TPABounds(tpa);
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
  b := TPABounds(tpa);
  SetLength(Result, b.Y2+1, b.X2+1);
  for p in tpa do Result[p.Y, p.X] := True;
end;

function AStarTPA(tpa: TPointArray; start, goal: TPoint; out totalDist: Double; diagonalTravel: Boolean = True; maxDistanceMultiplier: Int32 = 5): TPointArray;
  const OFFSETS: array[0..7] of TPoint = ((X:0; Y:-1),(X:-1; Y:0),(X:1; Y:0),(X:0; Y:1),(X:1; Y:-1),(X:-1; Y:1),(X:1; Y:1),(X:-1; Y:-1));
  type TNode = record Point: TPoint; Distance: Double; Priority: Double; end;
  var queue: array of TNode;
  var parents: T2DPointArray;

  function _Pop(): TNode;
  var
    i: Int32 = 0;
    child: Int32 = 1;
    next: Int32 = 2;
    hi: Int32;
  begin
    Result := queue[0];

    hi := High(queue);
    if hi < 1 then Exit;

    i := 0;
    child := 1;
    next := 2;
    // Min-heap deletion. I don't really understand this loop. Some kind of weird sorting
    while (child < hi) do
    begin
      if queue[hi].Priority <= queue[child].Priority then Break;
      if (next < hi) and (queue[next].Priority < queue[child].Priority) then child := next;

      queue[i] := queue[child];
      i := child;

      child := child * 2 + 1;
      next := child + 1;
    end;

    queue[i] := queue[hi];
    Delete(queue, hi, 1);
  end;

  function _BuildPath(): TPointArray;
  var
    tmp: TPoint;
    len: Int32 = 0;
  begin
    tmp := goal;

    while tmp <> start do
    begin
      Inc(len);
      SetLength(Result, len);
      Result[len-1] := tmp;
      tmp := parents[tmp.Y, tmp.X];
    end;

    Inc(len);
    SetLength(Result, len);
    Result[len-1] := tmp;
    TPAReverse(Result);
  end;

var
  current, next: TNode;
  b: TBox;
  matrix, visited: array of TBoolArray;
  path: TPointArray;
  p: TPoint;
  idx, i, hi: Int32;
  dist: Double;
begin
  b := TPABounds(tpa);
  SetLength(matrix, b.Y2+1, b.X2+1);

  for p in tpa do matrix[p.Y, p.X] := True;
  if not matrix[start.Y, start.X] then Exit;
  if not matrix[goal.Y, goal.X] then Exit;

  SetLength(visited, b.Y2+1, b.X2+1);
  SetLength(parents, b.Y2+1, b.X2+1);

  dist := DistanceBetween(start, goal);

  current.Point := start;
  current.Distance := 0;
  current.Priority := dist;

  visited[start.Y, start.X] := True;

  idx := Length(queue);
  SetLength(queue, idx + 1);
  queue[idx] := current;

  dist := maxDistanceMultiplier * dist;
  if diagonalTravel then hi := 7 else hi := 3;

  while Length(queue) > 0 do
  begin
    current := _Pop();
    //WriteLn('[',current.Point.X, ',', current.Point.Y, ']');

    if current.Point = goal then
    begin
      totalDist := current.Distance;
      Exit(_BuildPath());
    end;

    if current.Distance >= dist then Exit;

    for i := 0 to hi do
    begin
      next.Point := current.Point + OFFSETS[i];

      //try except seems to be faster than a bounds check lol.
      try if visited[next.Point.Y, next.Point.X] then Continue; except Continue; end;

      if not matrix[next.Point.Y, next.Point.X] then Continue;

      if i < 4 then next.Distance := Current.Distance + 1
      else           next.Distance := Current.Distance + Sqrt(2);

      next.Priority := next.Distance + DistanceBetween(next.Point, goal);

      visited[next.Point.Y, next.Point.X] := True;
      parents[next.Point.Y, next.Point.X] := current.Point;

      idx := Length(queue);
      SetLength(queue, idx + 1);
      queue[idx] := next;
    end;
  end;

end;


end.

