function [nomsColonnes, donnees] = lireFichierCSV(nomFichier)
    fichier = fopen(nomFichier, 'r');
    ligne = fgetl(fichier);
    
    % Lecture de la première ligne pour obtenir les noms des colonnes
    nomsColonnes = parseNomsColonnes(ligne);
    numColonnes = numel(nomsColonnes);
    
    % Initialisation de la matrice de données
    donnees = zeros(1, numColonnes);
    
    % Lecture des données du fichier
    ligne = fgetl(fichier);
    lineIdx = 0;
    while ischar(ligne)
        lineIdx = lineIdx + 1;
        valeurs = parseLigne(ligne);
        valeursNumeriques = str2double(valeurs);
        if size(donnees, 1) < lineIdx
            donnees = [donnees; zeros(size(donnees, 1), numColonnes)];
        end
        donnees(lineIdx, :) = valeursNumeriques;
        ligne = fgetl(fichier);
    end
    
    % Fermeture du fichier
    fclose(fichier);
    
    donnees = donnees(1:lineIdx, :);
end

function nomsColonnes = parseNomsColonnes(ligne)
    nomsColonnes = [];
    nomCol = '';
    i = 1;
    while i<=length(ligne)
        if ligne(i) == ','
            nomsColonnes = [nomsColonnes string(nomCol)];
            nomCol = '';
            i = i+1;
        elseif ligne(i) == '"'
            i = i+1;
            while ligne(i) ~= '"'
                nomCol = [nomCol ligne(i)];
                i = i+1;
            end
            nomsColonnes = [nomsColonnes string(nomCol)];
            nomCol = '';
            i = i+2; % pour sauter ",
        else
            nomCol = [nomCol ligne(i)];
            i = i+1;
        end
    end
    if length(nomCol)>0
        nomsColonnes = [nomsColonnes string(nomCol)];
    end
end


function valeurs = parseLigne(ligne)
    valeurs = [];
    val = '';
    for i=1:length(ligne)
        if ligne(i) == ','
            valeurs = [valeurs string(val)];
            val = '';
        else
            val = [val ligne(i)];
        end
    end
    valeurs = [valeurs string(val)];
end
