function validation = validateReferenceSafety(reference,map,config)
%VALIDATEREFERENCESAFETY Kiem tra swept footprint, ke ca pivot tai cho.
poses=[reference.x(:) reference.y(:) reference.theta(:)];
validation=evaluatePoseSequenceSafety(poses,map,config);
end
