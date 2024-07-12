library tpaex;

{$mode objfpc}{$H+}

uses
  sysutils, tpa, types;

{$I simbaplugin.inc}

procedure Lape_NRSplitTPA(const Params: PParamArray; const Result: Pointer); cdecl;
begin
  P2DPointArray(Result)^ := NRSplitTPA(PPointArray(Params^[0])^, PDouble(Params^[1])^);
end;

procedure Lape_NRClusterTPA(const Params: PParamArray; const Result: Pointer); cdecl;
begin
  P2DPointArray(Result)^ := NRClusterTPA(PPointArray(Params^[0])^, PDouble(Params^[1])^);
end;

procedure Lape_SkeletonTPA(const Params: PParamArray; const Result: Pointer); cdecl;
begin
  PPointArray(Result)^ := SkeletonTPA(PPointArray(Params^[0])^, PInt32(Params^[1])^, PInt32(Params^[2])^);
end;

procedure Lape_TPAMatrix(const Params: PParamArray; const Result: Pointer); cdecl;
begin
  P2DBoolArray(Result)^ := TPAMatrix(PPointArray(Params^[0])^);
end;

procedure Lape_AStarTPA(const Params: PParamArray; const Result: Pointer); cdecl;
begin
  PPointArray(Result)^ := AStarTPA(PPointArray(Params^[0])^, PPoint(Params^[1])^, PPoint(Params^[2])^, PDouble(Params^[3])^, PBoolean(Params^[4])^, PInt32(Params^[5])^);
end;

begin
  addGlobalFunc('function NRSplitTPA(const arr: TPointArray; dist: Double): T2DPointArray; native;', @Lape_NRSplitTPA);
  addGlobalFunc('function NRClusterTPA(const TPA: TPointArray; dist: Double): T2DPointArray; native;', @Lape_NRClusterTPA);
  addGlobalFunc('function SkeletonTPA(tpa: TPointArray; fMin, fMax: Int32): TPointArray; native;', @Lape_SkeletonTPA);
  addGlobalFunc('function TPAMatrix(tpa: TPointArray): T2DBoolArray; native;', @Lape_TPAMatrix);
  addGlobalFunc('function AStarTPA(tpa: TPointArray; start, goal: TPoint; out totalDist: Double; diagonalTravel: Boolean = True; maxDistanceMultiplier: Int32 = 5): TPointArray; native;', @Lape_AStarTPA);
end.
