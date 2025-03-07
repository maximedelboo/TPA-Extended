library tpaex;

{$mode objfpc}{$H+}

uses
  sysutils, tpa, types;

{$I simbaplugin.inc}

procedure Lape_NRSplitTPA(const Params: PParamArray; const Result: Pointer); cdecl;
begin
  P2DPointArray(Result)^ := NRSplitTPA(PPointArray(Params^[0])^, PSingle(Params^[1])^, PSingle(Params^[2])^);
end;

procedure Lape_FloodFillTPA(const Params: PParamArray; const Result: Pointer); cdecl;
begin
  P2DPointArray(Result)^ := FloodFillTPA(PPointArray(Params^[0])^);
end;

procedure Lape_NRClusterTPA(const Params: PParamArray; const Result: Pointer); cdecl;
begin
  P2DPointArray(Result)^ := NRClusterTPA(PPointArray(Params^[0])^, PSingle(Params^[1])^);
end;

procedure Lape_SkeletonTPA(const Params: PParamArray; const Result: Pointer); cdecl;
begin
  PPointArray(Result)^ := SkeletonTPA(PPointArray(Params^[0])^, PInt32(Params^[1])^, PInt32(Params^[2])^);
end;

procedure Lape_TPAMatrix(const Params: PParamArray; const Result: Pointer); cdecl;
begin
  P2DBoolArray(Result)^ := TPAMatrix(PPointArray(Params^[0])^);
end;

procedure Lape_AStarTPAEx(const Params: PParamArray; const Result: Pointer); cdecl;
begin
  PPointArray(Result)^ := AStarTPAEx(PPointArray(Params^[0])^, P2DFloatArray(Params^[1])^, PPoint(Params^[2])^, PPoint(Params^[3])^, PBoolean(Params^[4])^);
end;

procedure Lape_AStarTPA(const Params: PParamArray; const Result: Pointer); cdecl;
begin
  PPointArray(Result)^ := AStarTPA(PPointArray(Params^[0])^, PPoint(Params^[1])^, PPoint(Params^[2])^, PBoolean(Params^[3])^);
end;

begin
  addGlobalFunc('function NRSplitTPA(const arr: TPointArray; w, h: Single): T2DPointArray; native;', @Lape_NRSplitTPA);
  addGlobalFunc('function NRFloodFillTPA(const TPA: TPointArray; dist: Single): T2DPointArray; native;', @Lape_FloodFillTPA);
  addGlobalFunc('function NRClusterTPA(const TPA: TPointArray; dist: Single): T2DPointArray; native;', @Lape_NRClusterTPA);
  addGlobalFunc('function SkeletonTPA(tpa: TPointArray; fMin, fMax: Int32): TPointArray; native;', @Lape_SkeletonTPA);
  addGlobalFunc('function TPAMatrix(tpa: TPointArray): TBooleanMatrix; native;', @Lape_TPAMatrix);
  addGlobalFunc('function AStarTPAEx(tpa: TPointArray; out paths: TSingleMatrix; start, goal: TPoint; diagonalTravel: Boolean): TPointArray; native;', @Lape_AStarTPAEx);
  addGlobalFunc('function AStarTPA(tpa: TPointArray; start, goal: TPoint; diagonalTravel: Boolean): TPointArray; native;', @Lape_AStarTPA);
end.
