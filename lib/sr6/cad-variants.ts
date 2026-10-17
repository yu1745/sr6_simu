export const CAD_VARIANTS = {
  '8108': {label:'8108', data:'/sr6-cad.json', masses:'/sr6-mass-properties.json', download:'/sr6_mini_centered_step.zip'},
  '6010': {label:'6010', data:'/sr6-6010-cad.json', masses:'/sr6-6010-mass-properties.json', download:'/sr6_6010_centered_step.zip'},
} as const;
export type CadVariant = keyof typeof CAD_VARIANTS;
