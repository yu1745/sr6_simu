"use client";
import { useEffect, useState } from 'react';

// Restore after hydration; never overwrite saved values with the SSR defaults.
export function useStoredSetting<T>(key:string,initial:T,validate:(value:unknown)=>value is T){
  const [value,setValue]=useState(initial);
  const [loaded,setLoaded]=useState(false);
  useEffect(()=>{
    try{
      const raw=localStorage.getItem(key);
      if(raw!==null){const saved:unknown=JSON.parse(raw);if(validate(saved))setValue(saved);}
    }catch{/* Corrupt data or unavailable storage falls back to the default. */}
    setLoaded(true);
  },[key,validate]);
  useEffect(()=>{
    if(!loaded)return;
    try{localStorage.setItem(key,JSON.stringify(value));}catch{/* Settings remain usable without storage. */}
  },[key,loaded,value]);
  return [value,setValue] as const;
}
