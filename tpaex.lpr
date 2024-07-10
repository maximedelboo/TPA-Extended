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

begin
  addGlobalFunc('function NRSplitTPA(const arr: TPointArray; dist: Double): T2DPointArray; native;', @Lape_NRSplitTPA);
  addGlobalFunc('function NRClusterTPA(const TPA: TPointArray; dist: Double): T2DPointArray; native;', @Lape_NRClusterTPA);
  addGlobalFunc('function SkeletonTPA(tpa: TPointArray; fMin, fMax: Int32): TPointArray; native;', @Lape_SkeletonTPA);
end.
